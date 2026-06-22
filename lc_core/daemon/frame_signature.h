/**
 * @file   frame_signature.h
 * @brief  Content/metadata signatures for detecting whether a frame folder's
 *         key files changed since it was last solved (restart de-duplication).
 *
 *         Hybrid policy: small files (<= limit) are hashed by content (exact);
 *         large files (e.g. point clouds) use mtime+size (cheap, no full read).
 * @date   2026-06
 */

#ifndef FRAME_SIGNATURE_H
#define FRAME_SIGNATURE_H

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <glog/logging.h>

namespace lc_daemon {

// FNV-1a 64-bit hash of a file's full content. Returns "h<hex>" or "" on error.
inline std::string hashFileContent(const std::string& path) {
    std::ifstream fin(path, std::ios::binary);
    if (!fin) return "";
    uint64_t h = 1469598103934665603ULL;     // FNV offset basis
    char buf[65536];
    while (fin.read(buf, sizeof(buf)) || fin.gcount()) {
        std::streamsize n = fin.gcount();
        for (std::streamsize i = 0; i < n; ++i) {
            h ^= static_cast<unsigned char>(buf[i]);
            h *= 1099511628211ULL;            // FNV prime
        }
    }
    std::ostringstream os;
    os << "h" << std::hex << h;
    return os.str();
}

// "m<mtime>_<size>" from stat. Returns "" if the file is missing.
inline std::string statSignature(const std::string& path) {
    struct stat s;
    if (stat(path.c_str(), &s) != 0) return "";
    std::ostringstream os;
    os << "m" << static_cast<long long>(s.st_mtime) << "_"
       << static_cast<long long>(s.st_size);
    return os.str();
}

// One file's signature under the hybrid rule.
inline std::string fileSignature(const std::string& path, long size_limit_bytes) {
    struct stat s;
    if (stat(path.c_str(), &s) != 0) return "MISSING";
    if (s.st_size <= size_limit_bytes) {
        std::string h = hashFileContent(path);
        return h.empty() ? statSignature(path) : h;
    }
    return statSignature(path);
}

// Combined signature over a set of files (order-stable). Used for the first
// frame's pre-processing 3-set: change in ANY file -> different signature.
inline std::string combinedSignature(const std::vector<std::string>& paths,
                                      long size_limit_bytes) {
    std::ostringstream os;
    for (const auto& p : paths) {
        os << fileSignature(p, size_limit_bytes) << "|";
    }
    return os.str();
}

} // namespace lc_daemon

#endif
