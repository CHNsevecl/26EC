#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <cmath>
#include "include/QD4310.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "include/Myhailo.hpp"
#include "include/zmq_send.hpp"
#include <zmq.hpp>

#define addr1 0x01
#define addr2 0x02

double dx = 0.0;
double prev_dx = 0.0;
bool ready = false;
double rpm_f = 0.0;       // 平滑后的球速度 (px/ms, 即 dx 的时间导数)
double dt = 0.0;          // 帧间隔 (ms)
bool first_frame = true;
bool ball_lost = true;    // 当前帧是否丢失钢球
int round_ = 0;
std::mutex mtx;

std::condition_variable smessage;
cv::Mat frame;
std::mutex g_frame_mutex;
cv::Mat g_frame;

// 可调控制参数（全局共享，供 Controller 读取、visual 线程通过键盘调节）
double kp        = 0.002;    // 比例增益 (每 px 偏差输出弧度)
double kv        = 1.5;      // 速度阻尼增益 (每 px/ms 速度输出弧度), 接近时制动帮助停下
double DB        = 2.0;      // 到位死区 (px), 偏差小于该值输出 0
double STALL_MULT = 2.0;     // 卡滞增力倍率: 减小以免近处增力过大造成"输出太大"
double setpoint  = 320.0;    // 目标位置 (px, 默认图像中心)
const double V_STALL = 0.03; // 卡滞判定阈值 (px/ms): |v|<该值视为球卡住
double max_dangle  = 0.02;   // 每次控制允许的最大角度变化 (rad)，限制角速度抑制超调
double SOFT        = 0.004;  // 比例项软化系数: 大偏差时比例增益下降, 避免饱和猛推造成远距离震荡
double KV2         = 3.0;    // 非线性阻尼强度: 略增, 高速回冲时强减速帮助停下
double ki          = 0.04;   // 积分增益: 消除到位区静态误差
double I_ACT       = 40.0;   // 积分激活区(px): 仅偏差小于该值才累积/作用积分, 负责"近处挤精"消除静摩擦。
                             //   不宜过大, 否则积分在中段持续贡献恒力, 接近目标时无法停下(停不住)
double I_MAX       = 2.5;    // 积分上限(收敛积分最大力, 避免接近时持续推导致停不住)
double STALL_RANGE = 25.0;   // 卡滞增力范围(px): 仅在近处小偏差(≈到位区附近)且静止时, 用Kp×STALL_MULT突破静摩擦
                             //   不可过大, 否则远端大偏差静止会触发满幅增力导致输出过大


bool adjust_param_by_key(int key) {
    switch (key) {
        // kp
        case 'w': kp += 0.0005; return true;
        case 's': kp -= 0.0005; return true;
        // kv (速度阻尼)
        case 't': kv += 0.05; return true;
        case 'g': kv -= 0.05; return true;
        // DB 死区
        case 'e': DB += 1.0; return true;
        case 'd': DB -= 1.0; if (DB < 0.0) DB = 0.0; return true;
        // STALL_MULT 卡滞增力倍率
        case 'r': STALL_MULT += 0.5; return true;
        case 'f': STALL_MULT -= 0.5; if (STALL_MULT < 1.0) STALL_MULT = 1.0; return true;
        // setpoint 目标位置
        case 'a': setpoint -= 5.0; return true;
        case 'l': setpoint += 5.0; return true;
        // max_dangle 角度变化率上限
        case 'u': max_dangle += 0.005; return true;
        case 'j': max_dangle -= 0.005; if (max_dangle < 0.001) max_dangle = 0.001; return true;
        // SOFT 比例软化系数
        case 'i': SOFT += 0.001; return true;
        case 'k': SOFT -= 0.001; if (SOFT < 0.0) SOFT = 0.0; return true;
        // KV2 非线性阻尼
        case 'o': KV2 += 0.5; return true;
        case 'h': KV2 -= 0.5; if (KV2 < 0.0) KV2 = 0.0; return true;
        // ki 积分增益
        case 'p': ki += 0.01; return true;
        case 'q': ki -= 0.01; if (ki < 0.0) ki = 0.0; return true;
        // I_ACT 积分激活区
        case 'z': I_ACT += 10.0; return true;
        case 'x': I_ACT -= 10.0; if (I_ACT < DB) I_ACT = DB; return true;
        // I_MAX 积分上限
        case '[': I_MAX += 1.0; return true;
        case ']': I_MAX -= 1.0; if (I_MAX < 0.0) I_MAX = 0.0; return true;
        // STALL_RANGE 卡滞增力范围
        case 'c': STALL_RANGE += 10.0; return true;
        case 'v': STALL_RANGE -= 10.0; if (STALL_RANGE < DB) STALL_RANGE = DB; return true;
        default: return false;
    }
}

void print_params() {
    std::cout << "\n[params] kp=" << kp
              << " kv=" << kv
              << " DB=" << DB
              << " STALL_MULT=" << STALL_MULT
              << " setpoint=" << setpoint
              << " max_dangle=" << max_dangle 
              << " SOFT=" << SOFT
              << " KV2=" << KV2
              << " ki=" << ki
              << " I_ACT=" << I_ACT
              << " I_MAX=" << I_MAX
              << " STALL_RANGE=" << STALL_RANGE
              << std::endl;
}


int visual(){
    setenv("DISPLAY",":0",1);

    std::optional<HailoContext> hailo_context = Hailo_init(hef_path);
    if (!hailo_context) {
        std::cerr << "Error: Failed to initialize Hailo context." << std::endl;
        return -1;
    }
    int target_x = 320;
    int target_y = 320;

    //================初始化摄像头================
    std::string pipeline = 
    {
        "libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a ! "
        "video/x-raw,format=NV12,width=1280,height=640,framerate=50/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    };

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
    }
    
    auto prev_time = std::chrono::high_resolution_clock::now();

    while (true) {
        cv::Mat BGR_frame;
        cv::Mat RGB_frame;

        cap >> BGR_frame; // 读取一帧图像

        auto current_time = std::chrono::high_resolution_clock::now();
        dt = std::chrono::duration<double, std::milli>(current_time - prev_time).count();
        prev_time = current_time;

        if (BGR_frame.empty()) {
            std::cerr << "无法读取图像帧" << std::endl;
            return -1;
        }
        
        BGR_frame =  Stream_resize(BGR_frame);
        cv::cvtColor(BGR_frame, RGB_frame, cv::COLOR_BGR2RGB);
        std::optional<std::vector<Detection>> detections_opt = ParseDetections(*hailo_context, target_w, target_h, RGB_frame, class_names);
        bool ball_seen = false;
        for (const auto& detection : detections_opt.value_or(std::vector<Detection>{})) {
            if(detection.label == "steel_ball"){
                ball_seen = true;
                target_x = (detection.upper.x + detection.lower.x) / 2;
                target_y = (detection.upper.y + detection.lower.y) / 2;
                cv::Point center((detection.upper.x + detection.lower.x) / 2, (detection.upper.y + detection.lower.y) / 2);
                cv::circle(BGR_frame, center, 2, cv::Scalar(255, 0, 0), -1);
                cv::putText(BGR_frame,
                    detection.label + " " + cv::format("%.2f", detection.score),
                    cv::Point(center.x - 20, std::max(0, center.y - 10)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
            else if(detection.label == "pipe"){
                cv::rectangle(BGR_frame, detection.upper, detection.lower, cv::Scalar(0, 255, 0), 2);
                cv::putText(BGR_frame,
                    detection.label + " " + cv::format("%.2f", detection.score),
                    cv::Point(detection.upper.x, std::max(0, detection.upper.y - 10)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            if (ball_seen) {
                if (!first_frame) {
                    prev_dx = dx;
                    dx = setpoint - target_x;              // 相对设定点偏差 (px)
                    double rpm_raw = (dx - prev_dx) / dt;  // px/ms
                    if (std::abs(dx - prev_dx) > 40.0) rpm_raw = 0.0;  // 单帧跳变>40px 视为丢帧
                    rpm_raw = std::clamp(rpm_raw, -0.5, 0.5);          // 限幅 ±500px/s
                    rpm_f = 0.5 * rpm_f + 0.5 * rpm_raw;               // 低通平滑
                } else {
                    first_frame = false;
                }
                ball_lost = false;
            } else {
                ball_lost = true;   // 丢失: 保持上一 dx, 不归零到中心
                rpm_f *= 0.5;       // 速度衰减
            }
            ready = true;
            smessage.notify_one(); // 通知等待的线程
        }


        cv::imshow("Camera", BGR_frame); // 显示图像
        update_frame(BGR_frame, g_frame, g_frame_mutex);
        
        int key = cv::waitKey(1); // 按下ESC键退出，其他键调整参数
        if (key == 27) {
            break;
        }
        if (adjust_param_by_key(key)) {
            print_params();  // 打印调整后的参数，方便观察
        }
    }
    return 0;

}

void Controller(){
    QD4310 qd4310;

    qd4310.QD4310_Contol(addr1,QD4310_MODE_ENABLE,0);
    qd4310.QD4310_Contol(addr1,QD4310_MODE_ANGLE,qd4310.rad(0));

    // kp/kv/DB/STALL_MULT 为全局变量，可在 visual 线程用键盘实时调节
    std::cout << "键盘调试控制（在图像窗口内按键）：\n"
              << "  kp  +/- : W/S (步进0.0005)   kv  +/- : T/G (步进0.05)\n"
              << "  DB  +/- : E/D (px)          STALL+/-: R/F (倍率)\n"
              << "  setpoint+/-: A/L (5px)      角度限速+/-: U/J\n"
              << "  SOFT软化+/- : I/K           强阻尼+/- : O/H\n"
              << "  积分ki+/-   : P/Q           积分区+/- : Z/X  积分上限: [/]\n"
              << "  卡滞范围+/- : C/V (STALL_RANGE, 近处突破静摩擦)\n"
              << "  ESC 退出\n";
    print_params();

    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        smessage.wait(lock, [] { return ready; }); // 等待新一帧

        double e = dx;    // 球相对设定点偏差 (px)
        double v = rpm_f; // 平滑速度 (px/ms, 即 dx 的时间导数)

        // 到位区积分(消除静态误差), 设计要点对抗历史震问题:
        //  1) 仅 |e|<I_ACT(到位区)才累积/作用, 远端不攒积分 → 不产生积分windup
        //  2) 固定小步长累积 e*0.01 (与帧率无关), 慢帧不会让积分狂飙
        //  3) I_MAX 令 ki*I_MAX 远小于输出限幅(0.5) → 积分永远无法把管道推满到尽头
        //  4) 积分饱和冻结 + 进入死区 DB 清零, 消除残留累积
        static double integral = 0.0;

        double out = 0.0;
        if (!ball_lost && std::abs(e) > DB) {
            // 卡滞增力: 球在 STALL_RANGE 范围内且近静止时, 用 Kp×STALL_MULT 突破静摩擦
            //   范围需覆盖到位区并延伸到中段(≥I_ACT), 消除"盲区"导致球推不动停住
            bool stall = (std::abs(e) < STALL_RANGE && std::abs(v) < V_STALL);
            double kp_eff = stall ? kp * STALL_MULT : kp;

            // 比例软饱和: e/(1+SOFT*|e|) 令大偏差时有效增益下降, 避免饱和猛推造成远距离震荡
            double p_soft = e / (1.0 + SOFT * std::abs(e));

            // P-D 控制 + 非线性强阻尼:
            //   kv*v       线性阻尼
            //   KV2*v*|v|  二次阻尼, 高速回冲时强烈减速(抑制震荡)
            out = kp_eff * p_soft + kv * v + KV2 * v * std::abs(v);

            // 仅在到位区叠加积分(消除静摩擦残差), 且贡献量受 I_MAX 限制
            if (std::abs(e) < I_ACT) {
                out += ki * integral;
            }
            out = std::clamp(out, -0.5, 0.5);

            // 积分更新: 仅在到位区、且输出未饱和时累积
            bool sat_up   = (out >= 0.49 && e > 0);
            bool sat_down = (out <= -0.49 && e < 0);
            if (std::abs(e) < I_ACT && !sat_up && !sat_down) {
                integral += e * 0.01;              // 固定小步长, 与帧率无关
                integral = std::clamp(integral, -I_MAX, I_MAX);
            }
        }
        // |e|<=DB 或球丢失 → 输出 0 (管道放平) 且清零积分, 避免残留
        if (std::abs(e) <= DB || ball_lost) {
            integral = 0.0;
        }

        // 角速度限制 (rate limiter): 避免角度跳变过大, 抑制超调/振荡
        static bool first_cmd = true;
        static double last_cmd = 0.0;
        double new_cmd = out;
        if (!first_cmd) {
            double dcmd = std::clamp(new_cmd - last_cmd, -max_dangle, max_dangle);
            new_cmd = last_cmd + dcmd;
        }
        last_cmd = new_cmd;
        first_cmd = false;

        qd4310.QD4310_Contol(addr1,QD4310_MODE_ANGLE,qd4310.rad(new_cmd));
        ready = false; // 重置标志
    }
}

int main(){
    std::thread t1(visual);
    std::thread t2(Controller);
    std::thread t3(zmq_sender, std::ref(g_frame_mutex), std::ref(g_frame));
    t1.join();
    t2.join();
    t3.join();
    return 0;
}