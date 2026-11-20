/**
 * ========================================================
 *  宇泛灵猫 Cyvet 机器狗 —— 用 UniUbi Robot SDK 写的第一个可用程序
 * ========================================================
 *
 * 部署形态：外部 ARM64 Linux 主机（Jetson）远端 High-level 控制
 *   - 运行库目录：lib/aarch64_host/  （编译时必须 -DPLATFORM=aarch64_host）
 *   - 网卡        ：连着狗 Wi-Fi 的那块（本机是 wlP1p1s0）
 *   - 目标设备    ：必须显式给 SN，不能给 IP
 *
 * 控制链路：
 *   SDK -> RobotServer RPC -> MotionServer RPC（只有控制面，状态靠 query 取）
 *
 * 一次运行做的事：
 *   1) 初始化 SDK、连上狗、等能力表就绪
 *   2) 只读预检：电量、系统状态、当前动作、能力表（不发任何运动指令）
 *   3) 申请 High-level 控制权（startControl）
 *   4) standUp 站立 -> move 前进 -> move 转向 -> 清零速度 -> stopAction
 *   5) lieDown 趴下 -> 释放控制权 -> 断开
 *
 * 安全约定：
 *   - 遥控器必须先断开（长按 M 键，听到"遥控器连接已断开"），否则取权会被拒
 *   - 任何时刻 Ctrl-C 都会走"清速度 -> 停止动作 -> 释放控制权"的安全收尾
 *   - 所有动作都有秒数上限，不会无限行走
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "uniubi/robot_sdk/MotionHighLevelClient.h"
#include "uniubi/robot_sdk/MotionSdkService.h"

namespace {

using namespace uniubi::RobotSdk;
using Client = IMotionHighLevelClient;
using ClientPtr = std::shared_ptr<Client>;

/// 收到 SIGINT/SIGTERM 时置位，主流程在每次短睡眠里检查它
volatile std::sig_atomic_t gAbort = 0;

void onSignal(int) { gAbort = 1; }

/// 零速度参数：只清 walking 速度，不结束动作
constexpr const char* kStopVelocity =
    R"({"lineVelocityX":0.0,"lineVelocityY":0.0,"velocity":0.0})";

struct Options {
    std::string iface = "wlP1p1s0";  ///< DDS 网卡，远端模式必填
    std::string clientId = "cyvet-dog-demo";
    std::string deviceId;            ///< 目标机器人 SN，远端模式必填
    int32_t leaseMs = 60000;         ///< 控制权租约（<=0 由 SDK 填 60000）
    float vx = 0.25f;                ///< 前进线速度 m/s（保守值）
    double walkSeconds = 4.0;        ///< 前进持续秒数（0.25*4 ≈ 1 m）
    float vyaw = 0.4f;               ///< 转向角速度 rad/s
    double turnSeconds = 2.0;        ///< 转向持续秒数
    double standSeconds = 2.0;       ///< 站定后再动的等待时间
    double lieSeconds = 2.0;         ///< 趴下后等待收敛的时间
    bool doWalk = true;
    bool doTurn = true;
    bool readOnly = false;           ///< 只做只读预检，不取权不运动
    bool discover = false;           ///< 只发现设备然后退出
    bool skipCountdown = false;      ///< 跳过 5 秒倒计时
};

void printUsage(const char* program) {
    std::cout
        << "用法: " << program << " --device-id SN [选项]\n\n"
        << "必填（远端模式）:\n"
        << "  --device-id SN        目标机器人 SN（不是 IP）\n\n"
        << "常用选项:\n"
        << "  --iface IFACE         DDS 网卡，默认 wlP1p1s0\n"
        << "  --read-only           只做只读预检：电量/状态/能力，不取权不运动\n"
        << "  --discover            只发现设备（打印 SN 和 info JSON）后退出\n"
        << "  --vx MPS              前进线速度 m/s，默认 0.25\n"
        << "  --walk-seconds S      前进秒数，默认 4.0\n"
        << "  --vyaw RPS            转向角速度 rad/s，默认 0.4\n"
        << "  --turn-seconds S      转向秒数，默认 2.0\n"
        << "  --no-walk             跳过前进\n"
        << "  --no-turn             跳过转向\n"
        << "  --lease-ms MS         控制权租约，默认 60000\n"
        << "  --client-id ID        RPC 客户端标识\n"
        << "  --yes                 跳过 5 秒倒计时\n"
        << "  -h, --help            显示本帮助\n\n"
        << "示例:\n"
        << "  " << program << " --iface wlP1p1s0 --discover\n"
        << "  " << program << " --iface wlP1p1s0 --device-id E03C1CBAAC40A101 --read-only\n"
        << "  " << program << " --iface wlP1p1s0 --device-id E03C1CBAAC40A101 --yes\n";
}

bool parseDouble(const std::string& text, double& value) {
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (text.empty() || !end || *end != '\0') return false;
    value = parsed;
    return true;
}

bool parseOptions(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto needValue = [&](std::string& out) {
            if (++i >= argc) {
                std::cerr << "缺少 " << arg << " 的取值\n";
                return false;
            }
            out = argv[i];
            return true;
        };

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return false;
        }
        if (arg == "--read-only") { options.readOnly = true; continue; }
        if (arg == "--discover")  { options.discover = true; continue; }
        if (arg == "--no-walk")   { options.doWalk = false; continue; }
        if (arg == "--no-turn")   { options.doTurn = false; continue; }
        if (arg == "--yes")       { options.skipCountdown = true; continue; }

        if (arg == "--iface" || arg == "--client-id" || arg == "--device-id") {
            std::string value;
            if (!needValue(value)) return false;
            if (arg == "--iface")         options.iface = value;
            else if (arg == "--client-id") options.clientId = value;
            else                           options.deviceId = value;
            continue;
        }
        if (arg == "--lease-ms") {
            std::string value;
            if (!needValue(value)) return false;
            const long parsed = std::strtol(value.c_str(), nullptr, 10);
            if (parsed <= 0) {
                std::cerr << "非法 --lease-ms: " << value << '\n';
                return false;
            }
            options.leaseMs = static_cast<int32_t>(parsed);
            continue;
        }
        if (arg == "--vx" || arg == "--walk-seconds" || arg == "--vyaw" ||
            arg == "--turn-seconds" || arg == "--stand-seconds" ||
            arg == "--lie-seconds") {
            std::string value;
            if (!needValue(value)) return false;
            double parsed = 0.0;
            if (!parseDouble(value, parsed) || parsed < 0.0) {
                std::cerr << "非法 " << arg << ": " << value << '\n';
                return false;
            }
            if (arg == "--vx")                options.vx = static_cast<float>(parsed);
            else if (arg == "--walk-seconds") options.walkSeconds = parsed;
            else if (arg == "--vyaw")         options.vyaw = static_cast<float>(parsed);
            else if (arg == "--turn-seconds") options.turnSeconds = parsed;
            else if (arg == "--stand-seconds") options.standSeconds = parsed;
            else                              options.lieSeconds = parsed;
            continue;
        }

        std::cerr << "未知参数: " << arg << '\n';
        printUsage(argv[0]);
        return false;
    }

    if (options.discover && options.readOnly) {
        std::cerr << "--discover 和 --read-only 不能同时使用\n";
        return false;
    }
    if (!options.discover && options.deviceId.empty()) {
        std::cerr << "远端模式必须显式指定 --device-id SN（先用 --discover 查 SN）\n";
        return false;
    }
    return true;
}

/// 把 SDK 的错误码翻成可读名字，方便现场判断
const char* errorName(int32_t error) {
    switch (error) {
        case Client::kNone:               return "kNone（无错误）";
        case Client::kRpcConnectFailed:   return "kRpcConnectFailed（SDK 未初始化/连不上）";
        case Client::kRpcAcquireRejected: return "kRpcAcquireRejected（取权被拒：遥控器还连着或别人占着）";
        case Client::kRpcCallFailed:      return "kRpcCallFailed（RPC 失败/超时）";
        case Client::kSessionExpired:     return "kSessionExpired（租约到期）";
        case Client::kSessionRevoked:     return "kSessionRevoked（被别人接管）";
        case Client::kNotConnected:       return "kNotConnected（还没 connect）";
        case Client::kNotControlled:      return "kNotControlled（还没取到控制权）";
        case Client::kDataNotUpdate:      return "kDataNotUpdate（观测量未就绪）";
        case Client::kActionRejected:     return "kActionRejected（动作被拒）";
        case Client::kInvalidParam:       return "kInvalidParam（参数非法）";
        default:                          return "未知错误码";
    }
}

class DogDemo {
public:
    explicit DogDemo(const Options& options) : opts_(options) {}

    int run() {
        if (!initService()) return 1;

        if (opts_.discover) {
            const bool ok = runDiscovery();
            service_->shutdown();
            return ok ? 0 : 1;
        }

        client_ = Client::create(opts_.deviceId);
        if (!client_) {
            std::cerr << "[FAIL] 创建 High-level client 失败\n";
            service_->shutdown();
            return 1;
        }
        client_->setConnectCallback([this](Client::HighLevelState state,
                                           Client::HighLevelError error) {
            if (state == Client::kControlled) {
                std::cout << "[INFO] 已持有 High-level 控制权\n";
            } else if (state == Client::kConnected && error != Client::kNone) {
                std::cout << "[WARN] 控制权丢失: " << errorName(static_cast<int32_t>(error)) << '\n';
            }
        });

        if (!connect()) {
            cleanup(false);
            service_->shutdown();
            return 1;
        }

        // 只读预检：不发任何运动指令
        if (!preflight()) {
            cleanup(false);
            service_->shutdown();
            return 1;
        }
        if (opts_.readOnly) {
            std::cout << "[INFO] --read-only：预检完成，不取权、不运动，退出\n";
            cleanup(false);
            service_->shutdown();
            return 0;
        }

        if (!takeControl()) {
            cleanup(false);
            service_->shutdown();
            return 1;
        }

        const bool ok = runMotionScript();
        cleanup(ok);   // ok=true 走正常收尾（趴下），false 走安全收尾（保持站立）
        service_->shutdown();
        return ok ? 0 : 1;
    }

private:
    // ---------- 初始化 ----------

    bool initService() {
        service_ = IMotionSdkService::instance();
        if (!service_) {
            std::cerr << "[FAIL] IMotionSdkService::instance() 返回空\n";
            return false;
        }

        // 发现回调必须在 initialService 之前注册
        service_->setDiscoverCallback([this](const std::string& sn, const std::string& info) {
            std::cout << "  发现 SN=" << sn << " info=" << info << '\n';
            discovered_ = true;
        });

        // 网卡必须在 initialService 之前设置；板内单设备模式会忽略它
        service_->setNetworkInterface(opts_.iface.c_str());

        if (!service_->initialService(nullptr, opts_.clientId.c_str())) {
            std::cerr << "[FAIL] SDK 初始化失败（检查 env：source ~/env_uniubi.sh 后 "
                         "LD_LIBRARY_PATH 是否指向 lib/aarch64_host）\n";
            return false;
        }
        std::cout << "[PASS] SDK 初始化完成，版本 " << IMotionSdkService::version() << '\n';
        std::cout << "[INFO] 网卡=" << opts_.iface << '\n';
        return true;
    }

    bool runDiscovery() {
        std::cout << "[STEP] 设备发现（只读，5 秒窗口）...\n";
        if (!service_->discoverDevices(5000)) {
            std::cerr << "[FAIL] 发起发现失败\n";
            return false;
        }
        for (int i = 0; i < 50 && !discovered_ && !gAbort; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << (discovered_ ? "[PASS] 发现设备，用上面的 SN 做 --device-id\n"
                                  : "[FAIL] 没发现设备：确认网卡选对、狗 Wi-Fi 已连上\n");
        return discovered_;
    }

    // ---------- 连接与控制权 ----------

    bool connect() {
        std::cout << "[STEP] connect(lease=" << opts_.leaseMs << "ms) ...\n";
        if (!client_->connect(opts_.leaseMs)) {
            std::cout << "[FAIL] connect 失败，error=" << errorName(client_->getLastError()) << '\n';
            return false;
        }

        // connect 立即返回，能力表要等 RPC 通道就绪后重试
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        std::string capabilities;
        while (std::chrono::steady_clock::now() < deadline) {
            if (client_->getMotionCapabilities(capabilities)) {
                std::cout << "[PASS] 已连上设备，能力表就绪\n";
                return true;
            }
            if (client_->getLastError() != Client::kRpcConnectFailed) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::cout << "[FAIL] 能力表不可用，error=" << errorName(client_->getLastError()) << '\n';
        return false;
    }

    bool preflight() {
        std::cout << "[STEP] 只读预检\n";

        PowerObserved power = {};
        if (client_->getPowerInfo(&power, 5000)) {
            std::printf("  电量=%.1f%% 健康=%.1f%% 温度=%.1fC 电压=%.2fV 电流=%.2fA\n",
                        power.power, power.health, power.temper,
                        power.chargeVoltage, power.chargeCurrent);
            if (power.power < 30.0f) {
                std::cout << "  [WARN] 电量低于 30%，建议先充电再做动作\n";
            }
        } else {
            std::cout << "  [WARN] 电量读取失败: " << errorName(client_->getLastError()) << '\n';
        }

        std::string out;
        if (client_->querySystemStatus(out)) {
            std::cout << "  系统状态: " << out << '\n';
        } else {
            std::cout << "  [WARN] 系统状态读取失败: " << errorName(client_->getLastError()) << '\n';
        }

        if (client_->queryMotionState(out)) {
            std::cout << "  当前动作: " << (out.empty() ? "{}" : out)
                      << "  （{} 表示没有活动动作）\n";
        }

        if (client_->getMotionCapabilities(out)) {
            std::cout << "  能力表(动作/参数): " << out << '\n';
        }
        return true;
    }

    bool takeControl() {
        if (client_->getState() == Client::kControlled) return true;

        std::cout << "[STEP] 申请 High-level 控制权（遥控器必须已断开）\n";
        if (!client_->startControl(10000)) {
            std::cout << "[FAIL] startControl 未被受理，error="
                      << errorName(client_->getLastError()) << '\n';
            return false;
        }
        if (!waitForState(Client::kControlled, 10.0)) {
            std::cout << "[FAIL] 取权超时，error=" << errorName(client_->getLastError())
                      << "\n        多半是遥控器还连着：长按 M 键直到听到"
                         "“遥控器连接已断开”\n";
            return false;
        }
        std::cout << "[PASS] 控制权已拿到（此时还没有任何动作在跑）\n";
        return true;
    }

    // ---------- 动作脚本 ----------

    bool runMotionScript() {
        if (!opts_.skipCountdown) {
            std::cout << "[STEP] 5 秒后开始动作，Ctrl-C 可取消\n";
            if (!waitSeconds(5.0)) return false;
        }

        if (!step("站立", client_->standUp())) return false;
        if (!waitSeconds(opts_.standSeconds)) return false;

        if (opts_.doWalk) {
            std::cout << "[STEP] 前进 " << opts_.vx << " m/s x " << opts_.walkSeconds << " s\n";
            if (!step("前进", client_->move(opts_.vx, 0.0f, 0.0f))) return false;
            if (!waitSeconds(opts_.walkSeconds)) return false;
            if (!step("清零速度", client_->setActionParams(kStopVelocity))) return false;
        }

        if (opts_.doTurn) {
            std::cout << "[STEP] 原地转向 " << opts_.vyaw << " rad/s x " << opts_.turnSeconds
                      << " s\n";
            if (!step("转向", client_->move(0.0f, 0.0f, opts_.vyaw))) return false;
            if (!waitSeconds(opts_.turnSeconds)) return false;
            if (!step("清零速度", client_->setActionParams(kStopVelocity))) return false;
        }

        if (!step("停止动作", client_->stopAction())) return false;
        if (!waitSeconds(1.0)) return false;

        if (!step("趴下", client_->lieDown())) return false;
        if (!waitSeconds(opts_.lieSeconds)) return false;

        return true;
    }

    // ---------- 收尾 ----------

    /// ok=true：正常结束，已趴下；ok=false：中途失败/中断，保持站立并安全降级
    void cleanup(bool completed) {
        if (!client_) return;

        if (client_->getState() == Client::kControlled) {
            if (completed) {
                std::cout << "[STEP] 释放控制权\n";
            } else {
                std::cout << "[STEP] 安全收尾：清速度 -> 停止动作 -> 释放控制权"
                             "（狗保持站立，由你接管）\n";
                client_->setActionParams(kStopVelocity);
                client_->stopAction();
            }
            client_->releaseControl();
            waitForState(Client::kConnected, 3.0);
        }
        client_->disconnect();
        std::cout << "[PASS] 已断开\n";
    }

    // ---------- 工具 ----------

    /// 返回 false 表示被 Ctrl-C 打断
    bool waitSeconds(double seconds) {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < deadline) {
            if (gAbort) {
                std::cout << "\n[WARN] 收到中断信号，立即收尾\n";
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return !gAbort;
    }

    bool waitForState(int32_t state, double seconds) {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < deadline) {
            if (client_->getState() == state) return true;
            if (gAbort) return client_->getState() == state;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return client_->getState() == state;
    }

    bool step(const char* what, bool ok) {
        if (ok) {
            std::cout << "  [PASS] " << what << '\n';
            return true;
        }
        std::cout << "  [FAIL] " << what << "，error=" << errorName(client_->getLastError())
                  << '\n';
        return false;
    }

    Options opts_;
    IMotionSdkService* service_ = nullptr;
    ClientPtr client_;
    std::atomic<bool> discovered_{false};
};

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) {
        return (argc > 1 && (std::string(argv[1]) == "-h" ||
                             std::string(argv[1]) == "--help")) ? 0 : 2;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "=== Cyvet 机器狗 SDK 示例 ===\n";
    DogDemo demo(options);
    return demo.run();
}
