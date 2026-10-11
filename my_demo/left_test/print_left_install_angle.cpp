#include <cstdio>
#include <string>

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"

// 临时诊断程序：只登录左臂，读取并打印安装角，不 power_on / 不 enable / 不动机械臂。
// 用法：
//   ./print_left_install_angle [左臂IP]
// 默认左臂 IP 与 RC_ctrl.launch.py 一致：192.168.71.37

static const char *err_name(int ret)
{
    switch (ret)
    {
    case 0:
        return "ERR_SUCC";
    case -1:
        return "ERR_INVALID_HANDLER";
    case -2:
        return "ERR_INVALID_PARAMETER";
    case -3:
        return "ERR_COMMUNICATION_ERR";
    case -4:
        return "ERR_KINE_INVERSE_ERR";
    case -5:
        return "ERR_EMERGENCY_PRESSED";
    case -6:
        return "ERR_NOT_POWERED";
    case -7:
        return "ERR_NOT_ENABLED";
    case -8:
        return "ERR_DISABLE_SERVOMODE";
    case -9:
        return "ERR_NOT_OFF_ENABLE";
    case -10:
        return "ERR_PROGRAM_IS_RUNNING";
    case -11:
        return "ERR_CANNOT_OPEN_FILE";
    case -12:
        return "ERR_MOTION_ABNORMAL";
    default:
        return "UNKNOWN";
    }
}

int main(int argc, char **argv)
{
    std::string ip = (argc > 1) ? argv[1] : "192.168.71.37";

    std::printf("[diag] logging in to left arm: %s ...\n", ip.c_str());
    JAKAZuRobot robot;
    int ret = robot.login_in(ip.c_str());
    if (ret != ERR_SUCC)
    {
        std::printf("[diag] login_in FAILED ret=%d (%s)\n", ret, err_name(ret));
        return 1;
    }
    std::printf("[diag] login_in OK\n");

    Quaternion quat{};
    Rpy rpy{};
    ret = robot.get_installation_angle(&quat, &rpy);
    if (ret != ERR_SUCC)
    {
        std::printf("[diag] get_installation_angle FAILED ret=%d (%s)\n", ret, err_name(ret));
        robot.login_out();
        return 1;
    }

    std::printf("[diag] installation quaternion (s,x,y,z): %.9f, %.9f, %.9f, %.9f\n",
                quat.s, quat.x, quat.y, quat.z);
    std::printf("[diag] installation rpy (rad): rx=%.9f ry=%.9f rz=%.9f\n",
                rpy.rx, rpy.ry, rpy.rz);
    std::printf("[diag] installation rpy (deg): rx=%.6f ry=%.6f rz=%.6f\n",
                rpy.rx * 180.0 / 3.14159265358979323846,
                rpy.ry * 180.0 / 3.14159265358979323846,
                rpy.rz * 180.0 / 3.14159265358979323846);

    const bool all_zero =
        quat.s == 0.0 && quat.x == 0.0 && quat.y == 0.0 && quat.z == 0.0 &&
        rpy.rx == 0.0 && rpy.ry == 0.0 && rpy.rz == 0.0;
    if (all_zero)
    {
        std::printf("[diag] RESULT: installation angle is ZERO (no rotation configured on this controller)\n");
    }

    JointValue joint_pos{};
    ret = robot.get_joint_position(&joint_pos);
    if (ret == ERR_SUCC)
    {
        std::printf("[diag] joint position (deg):");
        for (int i = 0; i < 6; ++i)
        {
            std::printf(" j%d=%.4f", i + 1, joint_pos.jVal[i] * 180.0 / 3.14159265358979323846);
        }
        std::printf("\n");
    }
    else
    {
        std::printf("[diag] get_joint_position FAILED ret=%d (%s)\n", ret, err_name(ret));
    }

    CartesianPose tcp{};
    ret = robot.get_tcp_position(&tcp);
    if (ret == ERR_SUCC)
    {
        std::printf("[diag] tcp position (mm): x=%.3f y=%.3f z=%.3f\n",
                    tcp.tran.x, tcp.tran.y, tcp.tran.z);
        std::printf("[diag] tcp rpy (deg): rx=%.4f ry=%.4f rz=%.4f\n",
                    tcp.rpy.rx * 180.0 / 3.14159265358979323846,
                    tcp.rpy.ry * 180.0 / 3.14159265358979323846,
                    tcp.rpy.rz * 180.0 / 3.14159265358979323846);
    }
    else
    {
        std::printf("[diag] get_tcp_position FAILED ret=%d (%s)\n", ret, err_name(ret));
    }

    ret = robot.login_out();
    std::printf("[diag] login_out ret=%d (%s)\n", ret, err_name(ret));
    return ret == ERR_SUCC ? 0 : 1;
}
