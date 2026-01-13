#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <MocapApi.h>
#include <nlohmann/json.hpp>

#include "MocapData.h"
#include "librobot/include/robot.h"

using namespace MocapApi;

namespace {
std::string loadFile(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        std::cerr << "[Demo3] 无法打开文件: " << path << std::endl;
        return {};
    }
    std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    return content;
}

std::vector<std::string> getRetargetJointNames(const std::string& jsonStr) {
    std::vector<std::string> names;
    auto j = nlohmann::json::parse(jsonStr, nullptr, false);
    if (j.is_discarded()) return names;
    if (j.contains("retargetJoints") && j["retargetJoints"].is_array()) {
        for (const auto& item : j["retargetJoints"]) {
            if (item.contains("retargetJoint") && item["retargetJoint"].is_string()) {
                names.push_back(item["retargetJoint"].get<std::string>());
            }
        }
    }
    return names;
}

void printUsage() {
    std::cout << "用法: robot_api_demo3 <bvh路径> [retarget.json路径] [fps]" << std::endl;
}
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage();
        return 1;
    }

    std::string bvhPath = argv[1];
    std::string retargetPath = (argc >= 3) ? argv[2] : "retarget.json";
    int fps = (argc >= 4) ? std::max(1, atoi(argv[3])) : 60;
    float fixedDelta = 1.0f / static_cast<float>(fps);

    if (!std::filesystem::exists(bvhPath)) {
        std::cerr << "[Demo3] BVH 文件不存在: " << bvhPath << std::endl;
        return 1;
    }

    std::string retargetContent = loadFile(retargetPath);
    if (retargetContent.empty()) {
        std::cerr << "[Demo3] 读取 retarget 配置失败: " << retargetPath << std::endl;
        return 1;
    }

    std::ofstream logFile("robot_api_demo3_output.txt", std::ios::trunc);
    if (!logFile.is_open()) {
        std::cerr << "[Demo3] 无法创建输出文件 robot_api_demo3_output.txt" << std::endl;
        return 1;
    }

    std::vector<std::string> retargetNames = getRetargetJointNames(retargetContent);

    eba::Robot robot;
    if (!robot.Load(retargetContent.c_str())) {
        std::cerr << "[Demo3] Robot.Load 失败" << std::endl;
        return 1;
    }
    robot.SetFPS(fps);

    size_t frameIndex = 0;
    for (;; ++frameIndex) {
        std::vector<bvh::MocapData> frame;
        if (!bvh::LoadBvhFrame(bvhPath, frameIndex, frame))
            break;

        constexpr int kJointCount = MocapApi::JointTag_JointsCount; // 59
        std::vector<eba::MocapData> buffer(kJointCount);
        const size_t copyCount = std::min(frame.size(), buffer.size());
        for (size_t i = 0; i < copyCount; ++i) {
            buffer[i].localPos = frame[i].localPos;
            buffer[i].localRot = frame[i].localRot;
            buffer[i].worldPos = frame[i].worldPos;
            buffer[i].worldRot = frame[i].worldRot;
        }

        robot.Update(buffer.data());
        robot.Step(fixedDelta);

        float rx, ry, rz, rw;
        robot.GetRootRotation(&rx, &ry, &rz, &rw);
        float px, py, pz;
        robot.GetRootPosition(&px, &py, &pz);
        {
            std::ostringstream oss;
            oss.setf(std::ios::fixed);
            oss.precision(3);
            oss << "[Frame " << frameIndex << "] Root pos:(" << px << ' ' << py << ' ' << pz
                << ") rot:(" << rw << ' ' << rx << ' ' << ry << ' ' << rz << ")\n";
            const std::string line = oss.str();
            std::cout << line;
            logFile << line;
        }

        for (const auto& name : retargetNames) {
            float angle = 0.0f;
            if (robot.GetRetargetJointAngle(name, &angle) == MocapApi::Error_None) {
                std::ostringstream oss;
                oss.setf(std::ios::fixed);
                oss.precision(3);
                oss << "    " << name << " : " << angle << "\n";
                const std::string line = oss.str();
                std::cout << line;
                logFile << line;
             }
         }
     }

    std::ostringstream oss;
    oss << "BVH 播放结束，总帧数: " << frameIndex << '\n';
    const std::string summary = oss.str();
    std::cout << summary;
    logFile << summary;
    logFile.flush();
    return 0;
}
