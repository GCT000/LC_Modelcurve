/**
 * @file   yaml_patcher.h
 * @brief  YAML configuration patcher for LC-CurveModel daemon.
 *         Aligns the model1.yaml fields with whatever data_collector
 *         actually drops into each timestamp folder (base/data/<ts>/).
 * @date   2026-06
 */

#ifndef YAML_PATCHER_H
#define YAML_PATCHER_H

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include "pcd_bbox.h"

namespace lc_daemon {

// ── file-name conventions (basenames inside every timestamp folder) ──────────
// Collected by data_collector for EVERY frame:
constexpr const char* COLLECTED_PCD = "extracted.pcd";
constexpr const char* COLLECTED_IMG = "image.png";
// Pre-processing 3-set, present ONLY in the first-frame folder:
constexpr const char* PRE_PCD       = "1.pcd";              // -> lidar_points_path
constexpr const char* PRE_SELECTED  = "image_points_1.txt"; // -> selected_points
constexpr const char* PRE_ENDPOINT  = "endpoint.txt";       // -> end_point ("x,y,z")

// Parse "x,y,z" (comma separated) from endpoint.txt into 3 doubles.
inline bool readEndpoint(const std::string& folder, std::vector<double>& out) {
    std::string path = folder + "/" + PRE_ENDPOINT;
    std::ifstream fin(path);
    if (!fin) {
        LOG(ERROR) << "Cannot open endpoint file: " << path;
        return false;
    }
    std::string line;
    std::getline(fin, line);
    out.clear();
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        try {
            out.push_back(std::stod(tok));
        } catch (...) {
            LOG(ERROR) << "Bad number in endpoint.txt: '" << tok << "'";
            return false;
        }
    }
    if (out.size() != 3) {
        LOG(ERROR) << "endpoint.txt must have 3 values, got " << out.size()
                   << " in: " << path;
        return false;
    }
    return true;
}

// First frame: set all path fields + end_point, and CLEAR last_image_path so
// curve takes its first_time branch. selected_points / lidar_points_path /
// end_point come from the pre-processing 3-set in this folder.
inline bool patchYamlFirst(const std::string& yaml_path,
                           const std::string& folder) {
    std::vector<double> ep;
    if (!readEndpoint(folder, ep)) return false;

    YAML::Node config = YAML::LoadFile(yaml_path);

    config["raw_pcd_file"]       = folder + "/" + COLLECTED_PCD;
    config["image_path"]         = folder + "/" + COLLECTED_IMG;
    config["lidar_points_path"]  = folder + "/" + PRE_PCD;
    config["selected_points"]    = folder + "/" + PRE_SELECTED;
    config["last_image_path"]    = "";   // empty -> first_time in curve

    YAML::Node ep_node;
    ep_node.push_back(ep[0]);
    ep_node.push_back(ep[1]);
    ep_node.push_back(ep[2]);
    ep_node.SetStyle(YAML::EmitterStyle::Flow);
    config["end_point"] = ep_node;

    // rectang_size + xy_interval, derived from 1.pcd bounding box.
    // rectang_size = [x_min, x_max, y_min, y_max, z_min, z_max]
    // xy_interval  = min/max over { y_min, y_max, endpoint.y }
    BBox bb = computeBBox(folder + "/" + PRE_PCD);
    if (bb.valid) {
        YAML::Node rect;
        rect.push_back(bb.x_min); rect.push_back(bb.x_max);
        rect.push_back(bb.y_min); rect.push_back(bb.y_max);
        rect.push_back(bb.z_min); rect.push_back(bb.z_max);
        rect.SetStyle(YAML::EmitterStyle::Flow);
        config["rectang_size"] = rect;

        double ep_y = ep[1];   // middle value of endpoint.txt
        double lo = std::min({ bb.y_min, bb.y_max, ep_y });
        double hi = std::max({ bb.y_min, bb.y_max, ep_y });
        config["xy_interval"]["start"] = lo;
        config["xy_interval"]["end"]   = hi;
        LOG(INFO) << "[patch] rectang_size set, xy_interval start=" << lo
                  << " end=" << hi;
    } else {
        LOG(ERROR) << "[patch] bbox invalid, rectang_size/xy_interval left unchanged";
    }

    // per-frame outputs
    config["res_path"]           = folder + "/";
    config["matched_point_file"] = folder + "/matched_points.pcd";
    config["tunnel_cloud_file"]  = folder + "/filtered_cloud.pcd";
    config["point_txt_file"]     = folder + "/matched_points.txt";

    std::ofstream fout(yaml_path);
    fout << config;
    LOG(INFO) << "[patch] FIRST frame -> " << folder
              << "  end_point=[" << ep[0] << "," << ep[1] << "," << ep[2] << "]";
    return true;
}

// Subsequent frame: only the changing fields. last_image_path points to the
// previous frame's image so curve runs optical-flow tracking.
inline bool patchYamlSubsequent(const std::string& yaml_path,
                                const std::string& folder,
                                const std::string& prev_image) {
    YAML::Node config = YAML::LoadFile(yaml_path);

    config["raw_pcd_file"]    = folder + "/" + COLLECTED_PCD;
    config["image_path"]      = folder + "/" + COLLECTED_IMG;
    config["last_image_path"] = prev_image;

    // per-frame outputs
    config["res_path"]           = folder + "/";
    config["matched_point_file"] = folder + "/matched_points.pcd";
    config["tunnel_cloud_file"]  = folder + "/filtered_cloud.pcd";
    config["point_txt_file"]     = folder + "/matched_points.txt";

    std::ofstream fout(yaml_path);
    fout << config;
    LOG(INFO) << "[patch] SUBSEQUENT frame -> " << folder
              << "  last_image=" << prev_image;
    return true;
}

} // namespace lc_daemon

#endif
