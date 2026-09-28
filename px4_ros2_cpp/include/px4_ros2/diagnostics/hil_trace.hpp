#pragma once

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <rmw/types.h>
#include <unistd.h>

namespace px4_ros2::diagnostics {

/** Small, opt-in JSONL trace for bounded split-host HIL diagnostics. */
class HilTrace {
public:
    class Event {
    public:
        explicit Event(std::string type) : type_(std::move(type)) {}
        Event &text(const std::string &key, const std::string &value) { field(key, escape(value), false); return *this; }
        Event &number(const std::string &key, uint64_t value) { field(key, std::to_string(value), true); return *this; }
        Event &decimal(const std::string &key, double value) { std::ostringstream s; s << std::setprecision(17) << value; field(key, s.str(), true); return *this; }
        Event &boolean(const std::string &key, bool value) { field(key, value ? "true" : "false", true); return *this; }
        void commit() { if (committed_) return; HilTrace::write(type_, fields_); fields_.clear(); committed_ = true; }
        ~Event() { commit(); }

    private:
        void field(const std::string &key, const std::string &value, bool raw) {
            if (!fields_.empty()) fields_ += ',';
            fields_ += '"' + escape(key) + "\":";
            fields_ += raw ? value : '"' + value + '"';
        }
        std::string type_;
        std::string fields_;
        bool committed_{false};
    };

    static Event event(const std::string &type) { return Event(type); }

    static uint64_t currentThreadId()
    {
        return std::hash<std::thread::id>{}(std::this_thread::get_id());
    }

    static std::string gid(const rmw_gid_t &value)
    {
        std::ostringstream stream;
        stream << std::hex << std::setfill('0');
        for (size_t index = 0; index < RMW_GID_STORAGE_SIZE; ++index) {
            if (index != 0) stream << ':';
            stream << std::setw(2) << static_cast<unsigned>(value.data[index]);
        }
        return stream.str();
    }

private:
    static uint64_t process_start_monotonic_ns() {
        static const uint64_t value = []() -> uint64_t {
            std::ifstream input("/proc/self/stat");
            std::string stat;
            std::getline(input, stat);
            const auto command_end = stat.rfind(')');
            if (command_end == std::string::npos) {
                return uint64_t{0};
            }

            std::istringstream fields(stat.substr(command_end + 2));
            std::string field;
            for (int index = 0; index < 19; ++index) {
                if (!(fields >> field)) {
                    return uint64_t{0};
                }
            }
            uint64_t start_ticks = 0;
            if (!(fields >> start_ticks)) {
                return uint64_t{0};
            }
            const long ticks_per_second = ::sysconf(_SC_CLK_TCK);
            if (ticks_per_second <= 0) {
                return uint64_t{0};
            }
            return (start_ticks * 1000000000ULL) / static_cast<uint64_t>(ticks_per_second);
        }();
        if (value != 0) {
            return value;
        }
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    static std::string escape(const std::string &value) {
        std::string escaped;
        for (const char character : value) {
            switch (character) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += character; break;
            }
        }
        return escaped;
    }

    static void write(const std::string &type, const std::string &fields) {
        static std::atomic<uint64_t> event_count{0};
        constexpr uint64_t kMaxEvents = 200000;
        if (event_count.fetch_add(1, std::memory_order_relaxed) >= kMaxEvents) return;
        const char *path = std::getenv("III_HIL_DIAGNOSTIC_TRACE");
        if (path == nullptr || *path == '\0') return;
        static std::mutex mutex;
        static std::ofstream output;
        std::lock_guard<std::mutex> lock(mutex);
        if (!output.is_open()) {
            try {
                const std::filesystem::path trace_path(path);
                std::filesystem::create_directories(trace_path.parent_path());
                output.open(trace_path, std::ios::out | std::ios::app);
            } catch (...) { return; }
        }
        if (!output.good()) return;
        const auto monotonic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const auto process_generation = HilTrace::process_start_monotonic_ns();
        static const std::string process_boot_id = [] {
            std::ifstream input("/proc/sys/kernel/random/boot_id");
            std::string value;
            std::getline(input, value);
            return value;
        }();
        const auto thread_id = currentThreadId();
        const char *process = std::getenv("III_HIL_DIAGNOSTIC_PROCESS");
        output << "{\"monotonic_ns\":" << monotonic_ns
               << ",\"process_start_monotonic_ns\":" << process_generation
               << ",\"process_generation\":" << process_generation
               << ",\"process_boot_id\":\"" << escape(process_boot_id) << "\""
               << ",\"pid\":" << static_cast<long long>(::getpid())
               << ",\"thread_id\":" << thread_id
               << ",\"process\":\"" << escape(process == nullptr ? "unknown" : process)
               << "\",\"event\":\"" << escape(type) << "\""
               << (fields.empty() ? "" : "," + fields) << "}\n";
        output.flush();
    }
};

}  // namespace px4_ros2::diagnostics
