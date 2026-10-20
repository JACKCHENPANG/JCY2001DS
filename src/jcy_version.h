#pragma once

#include <stdint.h>

// OTA 版本必须采用 V<major>.<minor>.<patch>-A<build>，例如 V0.8.4-A45。
// 不接受自由文本，避免把字符串字典序误当成版本序（例如 A10/A9）。
namespace jcy_version {

struct Version {
    uint32_t major = 0;
    uint32_t minor = 0;
    uint32_t patch = 0;
    uint32_t build = 0;
};

inline bool parse_number(const char *&cursor, uint32_t &value) {
    if (cursor == nullptr || *cursor < '0' || *cursor > '9') return false;

    uint32_t parsed = 0;
    do {
        const uint32_t digit = static_cast<uint32_t>(*cursor - '0');
        // 拒绝溢出版本号，不能把异常清单误判为可升级版本。
        if (parsed > (UINT32_MAX - digit) / 10U) return false;
        parsed = parsed * 10U + digit;
        ++cursor;
    } while (*cursor >= '0' && *cursor <= '9');

    value = parsed;
    return true;
}

inline bool parse(const char *text, Version &version) {
    if (text == nullptr || (text[0] != 'V' && text[0] != 'v')) return false;

    const char *cursor = text + 1;
    Version parsed;
    if (!parse_number(cursor, parsed.major) || *cursor++ != '.' ||
        !parse_number(cursor, parsed.minor) || *cursor++ != '.' ||
        !parse_number(cursor, parsed.patch) || *cursor++ != '-' ||
        (*cursor != 'A' && *cursor != 'a')) {
        return false;
    }
    ++cursor;
    if (!parse_number(cursor, parsed.build) || *cursor != '\0') return false;

    version = parsed;
    return true;
}

// 返回 >0 表示 lhs 更新，<0 表示 lhs 更旧，0 表示完全相同。
inline int compare(const Version &lhs, const Version &rhs) {
    if (lhs.major != rhs.major) return lhs.major > rhs.major ? 1 : -1;
    if (lhs.minor != rhs.minor) return lhs.minor > rhs.minor ? 1 : -1;
    if (lhs.patch != rhs.patch) return lhs.patch > rhs.patch ? 1 : -1;
    if (lhs.build != rhs.build) return lhs.build > rhs.build ? 1 : -1;
    return 0;
}

inline bool is_strictly_newer(const char *candidate, const char *current) {
    Version candidate_version;
    Version current_version;
    return parse(candidate, candidate_version) &&
           parse(current, current_version) &&
           compare(candidate_version, current_version) > 0;
}

}  // namespace jcy_version
