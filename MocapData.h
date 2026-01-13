#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

namespace bvh {
struct MocapData
{
    glm::quat localRot;
    glm::vec3 localPos;
    glm::quat worldRot;
    glm::vec3 worldPos;
};

struct BvhJointInfo {
    std::string name;
    int channelStart = -1;
    int channelCount = 0;
    std::vector<std::string> channelNames; // e.g. "Xposition","Yposition","Zposition","Zrotation","Xrotation","Yrotation"
};

static inline glm::quat axisAngleToQuat(char axis, float deg) {
    float rad = glm::radians(deg);
    if (axis == 'X') return glm::angleAxis(rad, glm::vec3(1.0f,0.0f,0.0f));
    if (axis == 'Y') return glm::angleAxis(rad, glm::vec3(0.0f,1.0f,0.0f));
    return glm::angleAxis(rad, glm::vec3(0.0f,0.0f,1.0f));
}

// 读取指定帧（frameIndex，从0开始），返回每个 joint 的 MocapData（按 BVH 顺序）
bool LoadBvhFrame(const std::string& path, size_t frameIndex, std::vector<MocapData>& outFrame)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    std::string line;
    std::vector<BvhJointInfo> joints;
    joints.reserve(128);

    // 解析 HIERARCHY，收集 joints 与 channels 顺序
    int runningChannelIndex = 0;
    std::vector<std::string> tokens;
    auto nextToken = [&](std::istringstream &ss)->std::string{ std::string t; ss >> t; return t; };

    // We will parse until "MOTION"
    while (std::getline(ifs, line)) {
        if (line.find("MOTION") != std::string::npos) break;
        std::istringstream ss(line);
        std::string first;
        ss >> first;
        if (first == "ROOT" || first == "JOINT") {
            std::string jointName;
            ss >> jointName;
            BvhJointInfo info;
            info.name = jointName;
            joints.push_back(info);
        } else if (first == "CHANNELS") {
            if (joints.empty()) return false;
            int n;
            ss >> n;

            joints.back().channelStart = runningChannelIndex;
            joints.back().channelCount = n;
            runningChannelIndex += n;
            for (int i = 0; i < n; ++i) {
                std::string ch;
                ss >> ch;
                joints.back().channelNames.push_back(ch); // keep order
            }
        } else {
            // ignore other lines in hierarchy (OFFSET, {, }, End Site...)
        }
    }

    // 到这里 ifs 已经停在 MOTION 后一行
    // 读取 Frames: 和 Frame Time:
    size_t frames = 0;
    double frameTime = 0.0;
    while (std::getline(ifs, line)) {
        if (line.size() == 0) continue;
        if (line.find("Frames:") != std::string::npos) {
            std::istringstream ss(line);
            std::string tmp; ss >> tmp >> frames;
        } else if (line.find("Frame Time:") != std::string::npos) {
            std::istringstream ss(line);
            std::string tmp; ss >> tmp >> tmp >> frameTime;
            break;
        }
    }
    if (frames == 0) return false;
    if (frameIndex >= frames) return false;

    // 跳到目标帧行（每帧一行或可能换行——这里假设每帧一行数字）
    size_t cur = 0;
    std::string frameLine;
    while (std::getline(ifs, frameLine)) {
        if (frameLine.size() == 0) continue;
        if (cur == frameIndex) break;
        ++cur;
    }
    if (frameLine.empty()) return false;

    // tokenize numbers
    std::istringstream fss(frameLine);
    std::vector<double> values;
    double v;
    while (fss >> v) values.push_back(v);

    // 如果数据跨多行（channel很多），需要补读直到足够
    while (values.size() < (size_t)runningChannelIndex && std::getline(ifs, line)) {
        std::istringstream extra(line);
        while (extra >> v) values.push_back(v);
    }
    if (values.size() < (size_t)runningChannelIndex) return false;

    // 构建 outFrame
    size_t jointCount = joints.size();
    outFrame.clear();
    outFrame.resize(jointCount);
    for (size_t ji = 0; ji < jointCount; ++ji) {
        const auto &info = joints[ji];
        glm::vec3 pos(0.0f);
        glm::quat rot = glm::quat(1.0f,0.0f,0.0f,0.0f);

        // BVH rotation composition: must apply rotations in the order channels are listed.
        glm::quat qAccum = glm::quat(1.0f,0.0f,0.0f,0.0f);

        for (int ci = 0; ci < info.channelCount; ++ci) {
            const std::string &cname = info.channelNames[ci];
            double val = values[info.channelStart + ci]; // degrees for rotation, units for pos
            if (cname == "Xposition") {
                pos.x = float(val);
            } else if (cname == "Yposition") {
                pos.y = float(val);
            } else if (cname == "Zposition") {
                pos.z = float(val);
            } else if (cname == "Xrotation" || cname == "Xrot") {
                // apply rotation around X by val degrees
                glm::quat q = axisAngleToQuat('X', float(val));
                qAccum =  qAccum* q; // note multiplication order matters
            } else if (cname == "Yrotation" || cname == "Yrot") {
                glm::quat q = axisAngleToQuat('Y', float(val));
                qAccum = qAccum*q;
            } else if (cname == "Zrotation" || cname == "Zrot") {
                glm::quat q = axisAngleToQuat('Z', float(val));
                qAccum = qAccum * q;
            }
        }

        // 如果 BVH 是以 cm 为单位，而你的 Robot 使用 m，可在这里转换 pos *= 0.01f;
        outFrame[ji].localPos = pos;        // 可能需要缩放/坐标转换
        outFrame[ji].localRot = qAccum;
        outFrame[ji].worldPos = outFrame[ji].localPos; // 若需要世界坐标，应由层级关系计算
        outFrame[ji].worldRot = outFrame[ji].localRot;
    }

    return true;
}

} // namespace bvh