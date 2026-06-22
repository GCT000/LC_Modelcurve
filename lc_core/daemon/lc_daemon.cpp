/**
 * @file   lc_daemon.cpp
 * @brief  Orchestrator: watch base/data for timestamp folders produced by
 *         data_collector, pick the first frame (newest folder that already has
 *         the pre-processing 3-set), then solve every subsequent frame in
 *         chronological order by invoking `curve`.
 * @date   2026-06
 *
 * Hardening for long-running operation:
 *   1. Sliding window: keep only the newest N frame folders on disk; the first
 *      frame is NEVER deleted (subsequent frames' yaml still reference its
 *      pre-processing files).
 *   2. Restart de-dup: a persisted state file records each solved folder's
 *      signature; on restart, unchanged folders are skipped. If the FIRST
 *      frame's pre-processing files changed (re-processed), the whole pipeline
 *      is reset and re-solved from the first frame.
 *   3. curve timeout: a frame whose curve runs longer than the limit is killed
 *      so it cannot stall the pipeline.
 *   4. Log rotation: the daemon's own log dir is cleared every N days.
 */

#include <sys/stat.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <regex>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <chrono>
#include <thread>
#include <fstream>
#include <filesystem>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "yaml_patcher.h"
#include "frame_signature.h"

namespace fs = std::filesystem;

DEFINE_string(data_root, "/home/gct/LC_Modelcurve/data", "Data root directory to monitor");
DEFINE_string(yaml_path, "/home/gct/LC_Modelcurve/config/whu/model1.yaml", "YAML config file");
DEFINE_string(curve_bin, "/home/gct/LC_Modelcurve/bin/curve", "Curve executable path");
DEFINE_string(daemon_log_dir, "/home/gct/LC_Modelcurve/data/lc_daemon_logs", "Daemon log directory");
DEFINE_string(state_file, "/home/gct/LC_Modelcurve/data/.lc_state", "Persisted solved-frame state");
DEFINE_int32(poll_sec, 2, "Polling interval in seconds");
DEFINE_int32(ready_timeout_sec, 120, "Max seconds to wait for a frame's collected files");
DEFINE_int32(keep_frames, 50, "Sliding window: max frame folders to keep on disk");
DEFINE_int32(curve_timeout_sec, 1200, "Max seconds a single curve run may take (20 min)");
DEFINE_int32(log_clear_days, 7, "Clear daemon log dir every N days");
DEFINE_double(hash_size_limit_mb, 1.0, "Files <= this size are signed by content hash");

// data_collector names folders "yy-mm-dd-HH-MM-SS" (2-digit year, 6 fields).
static const std::regex kTsPattern(R"(^\d{2}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}$)");

static bool isTimestamp(const std::string& name) {
    return std::regex_match(name, kTsPattern);
}

// Sorted ascending == chronological (timestamp format is lexicographic).
static std::vector<std::string> listFrameFolders(const std::string& root) {
    std::vector<std::string> out;
    std::error_code ec;
    for (auto& e : fs::directory_iterator(root, ec)) {
        if (ec) break;
        if (!e.is_directory()) continue;
        std::string name = e.path().filename().string();
        if (isTimestamp(name)) out.push_back(name);
    }
    std::sort(out.begin(), out.end());
    return out;
}

static bool fileExists(const std::string& p) {
    struct stat s;
    return stat(p.c_str(), &s) == 0;
}

static bool hasPreprocess(const std::string& folder) {
    return fileExists(folder + "/" + lc_daemon::PRE_PCD) &&
           fileExists(folder + "/" + lc_daemon::PRE_SELECTED) &&
           fileExists(folder + "/" + lc_daemon::PRE_ENDPOINT);
}

static bool hasCollected(const std::string& folder) {
    return fileExists(folder + "/" + lc_daemon::COLLECTED_PCD) &&
           fileExists(folder + "/" + lc_daemon::COLLECTED_IMG);
}

static bool waitForCollected(const std::string& folder) {
    for (int i = 0; i < FLAGS_ready_timeout_sec; ++i) {
        if (hasCollected(folder)) return true;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    LOG(ERROR) << "Timeout waiting for collected files in: " << folder;
    return false;
}

static long hashLimitBytes() {
    return static_cast<long>(FLAGS_hash_size_limit_mb * 1024 * 1024);
}

// Signature of the first frame = its pre-processing 3-set.
static std::string firstFrameSignature(const std::string& folder) {
    return lc_daemon::combinedSignature(
        { folder + "/" + lc_daemon::PRE_PCD,
          folder + "/" + lc_daemon::PRE_SELECTED,
          folder + "/" + lc_daemon::PRE_ENDPOINT },
        hashLimitBytes());
}

// Signature of a subsequent frame = its collected pair.
static std::string frameSignature(const std::string& folder) {
    return lc_daemon::combinedSignature(
        { folder + "/" + lc_daemon::COLLECTED_PCD,
          folder + "/" + lc_daemon::COLLECTED_IMG },
        hashLimitBytes());
}

// ── persisted state: folder name -> signature at solve time ──────────────────
static std::map<std::string, std::string> loadState() {
    std::map<std::string, std::string> st;
    std::ifstream fin(FLAGS_state_file);
    std::string line;
    while (std::getline(fin, line)) {
        auto tab = line.find('\t');
        if (tab == std::string::npos) continue;
        st[line.substr(0, tab)] = line.substr(tab + 1);
    }
    return st;
}

static void saveState(const std::map<std::string, std::string>& st) {
    std::string tmp = FLAGS_state_file + ".tmp";
    {
        std::ofstream fout(tmp, std::ios::trunc);
        for (const auto& kv : st) fout << kv.first << "\t" << kv.second << "\n";
    }
    std::error_code ec;
    fs::rename(tmp, FLAGS_state_file, ec);   // atomic replace
}

// ── run curve with a hard timeout ────────────────────────────────────────────
static bool runCurve(const std::string& log_dir) {
    fs::create_directories(log_dir);
    pid_t pid = fork();
    if (pid == 0) {
        std::string yaml_arg = "-yaml=" + FLAGS_yaml_path;
        std::string dir_arg  = "-dir=" + log_dir;
        char* args[] = {
            const_cast<char*>(FLAGS_curve_bin.c_str()),
            const_cast<char*>(yaml_arg.c_str()),
            const_cast<char*>(dir_arg.c_str()),
            const_cast<char*>("-visualize=false"),
            nullptr
        };
        execv(FLAGS_curve_bin.c_str(), args);
        LOG(ERROR) << "execv failed: " << strerror(errno);
        _exit(EXIT_FAILURE);
    }
    if (pid < 0) {
        LOG(ERROR) << "fork failed: " << strerror(errno);
        return false;
    }

    // parent: poll for completion up to the timeout
    int waited = 0;
    while (waited < FLAGS_curve_timeout_sec) {
        int status = 0;
        pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) {
            if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
                LOG(INFO) << "curve completed successfully";
                return true;
            }
            LOG(ERROR) << "curve failed, exit="
                       << (WIFEXITED(status) ? WEXITSTATUS(status) : -1);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++waited;
    }

    // timed out: terminate, then hard-kill if needed
    LOG(ERROR) << "curve exceeded " << FLAGS_curve_timeout_sec
               << "s, killing pid " << pid;
    kill(pid, SIGTERM);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (waitpid(pid, nullptr, WNOHANG) != pid) {
        kill(pid, SIGKILL);
        waitpid(pid, nullptr, 0);
    }
    return false;
}

// ── sliding window: keep newest N, but never delete the first frame ──────────
static void enforceWindow(const std::string& first_folder,
                          std::map<std::string, std::string>& state) {
    auto folders = listFrameFolders(FLAGS_data_root);
    int over = static_cast<int>(folders.size()) - FLAGS_keep_frames;
    if (over <= 0) return;
    for (int i = 0; i < static_cast<int>(folders.size()) && over > 0; ++i) {
        const std::string& name = folders[i];
        if (name == first_folder) continue;       // protected
        std::error_code ec;
        fs::remove_all(FLAGS_data_root + "/" + name, ec);
        if (!ec) {
            LOG(INFO) << "[window] removed old frame: " << name;
            state.erase(name);
            --over;
        }
    }
}

// Remove the global optical-flow seed so the first frame is unambiguous.
static void clearCurvePointFile() {
    try {
        YAML::Node cfg = YAML::LoadFile(FLAGS_yaml_path);
        if (cfg["curve_point_file"]) {
            std::string f = cfg["curve_point_file"].as<std::string>();
            std::error_code ec;
            if (fs::exists(f) && fs::remove(f, ec))
                LOG(INFO) << "Cleared stale curve_point_file: " << f;
        }
    } catch (...) {}
}

// ── weekly log clearing ──────────────────────────────────────────────────────
static void maybeClearLogs(std::chrono::steady_clock::time_point& last_clear) {
    auto now = std::chrono::steady_clock::now();
    auto days = std::chrono::duration_cast<std::chrono::hours>(now - last_clear).count() / 24;
    if (days < FLAGS_log_clear_days) return;
    google::FlushLogFiles(google::INFO);
    std::error_code ec;
    for (auto& e : fs::directory_iterator(FLAGS_daemon_log_dir, ec)) {
        if (ec) break;
        // keep the currently-open log; remove rotated/old ones
        fs::remove(e.path(), ec);
    }
    last_clear = now;
    LOG(INFO) << "[log] cleared daemon log dir (>= " << FLAGS_log_clear_days << " days)";
}

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    fs::create_directories(FLAGS_daemon_log_dir);
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel = google::INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = FLAGS_daemon_log_dir;
    google::SetLogFilenameExtension(".log");

    LOG(INFO) << "LC-CurveModel orchestrator started";
    LOG(INFO) << "Monitoring: " << FLAGS_data_root << "  (poll " << FLAGS_poll_sec
              << "s, keep " << FLAGS_keep_frames << " frames, curve timeout "
              << FLAGS_curve_timeout_sec << "s)";

    auto state = loadState();
    auto last_log_clear = std::chrono::steady_clock::now();

    // ── Phase 1: find the first frame (newest folder with the 3-set) ─────────
    std::string first_folder;
    while (first_folder.empty()) {
        auto folders = listFrameFolders(FLAGS_data_root);
        for (auto it = folders.rbegin(); it != folders.rend(); ++it) {
            if (hasPreprocess(FLAGS_data_root + "/" + *it)) {
                first_folder = *it;
                LOG(INFO) << "First frame selected: " << first_folder
                          << " (older folders ignored)";
                break;
            }
        }
        if (first_folder.empty()) {
            LOG(INFO) << "No pre-processed folder yet, waiting...";
            std::this_thread::sleep_for(std::chrono::seconds(FLAGS_poll_sec));
        }
    }

    // ── Phase 2: solve the first frame (unless unchanged since last run) ─────
    std::string first_path = FLAGS_data_root + "/" + first_folder;
    std::string first_sig  = firstFrameSignature(first_path);
    bool first_reprocessed = (state.count(first_folder) == 0) ||
                             (state[first_folder] != first_sig);

    if (first_reprocessed) {
        // re-processed (or never solved) -> reset the whole pipeline
        LOG(INFO) << "First frame is new/re-processed -> resetting pipeline";
        state.clear();
        if (!waitForCollected(first_path)) {
            LOG(FATAL) << "First frame has no collected files: " << first_path;
        }
        clearCurvePointFile();
        if (!lc_daemon::patchYamlFirst(FLAGS_yaml_path, first_path)) {
            LOG(FATAL) << "Failed to patch yaml for first frame: " << first_path;
        }
        runCurve(first_path + "/log");
        state[first_folder] = first_sig;
        saveState(state);
    } else {
        LOG(INFO) << "First frame unchanged, skipping re-solve: " << first_folder;
    }

    std::string last_solved = first_folder;
    std::string prev_image  = first_path + "/" + lc_daemon::COLLECTED_IMG;
    // advance watermark past any already-solved subsequent frames
    for (const auto& kv : state)
        if (kv.first > last_solved) last_solved = kv.first;

    enforceWindow(first_folder, state);
    saveState(state);

    // ── Phase 3: solve every newer frame in chronological order ──────────────
    while (true) {
        auto folders = listFrameFolders(FLAGS_data_root);
        for (const auto& name : folders) {
            if (name <= first_folder) continue;            // first frame or older
            std::string path = FLAGS_data_root + "/" + name;

            std::string sig = frameSignature(path);
            if (state.count(name) && state[name] == sig) continue;  // already solved, unchanged
            if (name <= last_solved && state.count(name)) continue;

            LOG(INFO) << "New frame detected: " << name;
            if (!waitForCollected(path)) {
                last_solved = std::max(last_solved, name);
                continue;
            }
            sig = frameSignature(path);   // recompute now that files are ready
            if (lc_daemon::patchYamlSubsequent(FLAGS_yaml_path, path, prev_image)) {
                if (runCurve(path + "/log")) {
                    prev_image = path + "/" + lc_daemon::COLLECTED_IMG;
                }
                state[name] = sig;
                saveState(state);
                enforceWindow(first_folder, state);
                saveState(state);
            }
            last_solved = std::max(last_solved, name);
        }
        maybeClearLogs(last_log_clear);
        std::this_thread::sleep_for(std::chrono::seconds(FLAGS_poll_sec));
    }
    return 0;
}
