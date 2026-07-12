#include <rcl_logging_interface/rcl_logging_interface.h>
#include <rcutils/logging.h>

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <memory>
#include <fstream>
#include <unordered_map>
#include <filesystem>
#include <mutex>
#include <cstdlib>

static std::shared_ptr<spdlog::logger> g_logger;
static std::once_flag g_init_flag;

// ---------------- 配置解析 ----------------
static std::unordered_map<std::string, std::string>
parse_config(const std::string & file)
{
  std::unordered_map<std::string, std::string> cfg;
  std::ifstream f(file);
  std::string line;

  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {continue;}

    auto pos = line.find('=');
    if (pos == std::string::npos) {continue;}

    cfg[line.substr(0, pos)] = line.substr(pos + 1);
  }

  return cfg;
}

// 展开路径中的 ~ 为 $HOME
static std::string expand_tilde(const std::string & path)
{
  if (path.empty() || path[0] != '~') {return path;}
  const char * home = std::getenv("HOME");
  if (!home) {return path;}
  return std::string(home) + path.substr(1);
}

// 从 rcutils 格式化消息 "[LEVEL] [timestamp] [name]: actual" 中提取实际内容
static const char * extract_msg(const char * name, const char * msg)
{
  if (!name || !msg) {return msg;}
  // 查找 "[name]: " 子串，取其后的内容
  std::string_view sv(msg);
  std::string needle = std::string("[") + name + "]: ";
  auto pos = sv.find(needle);
  if (pos != std::string_view::npos) {
    return msg + pos + needle.size();
  }
  return msg;
}

static spdlog::level::level_enum str_to_level(const std::string & s)
{
  if (s == "debug") {return spdlog::level::debug;}
  if (s == "info") {return spdlog::level::info;}
  if (s == "warn") {return spdlog::level::warn;}
  if (s == "error") {return spdlog::level::err;}
  if (s == "fatal") {return spdlog::level::critical;}
  return spdlog::level::info;
}

// ---------------- ROS2 logging 接口 ----------------
extern "C"
{

rcl_logging_ret_t rcl_logging_external_initialize(
  const char * config_file,
  rcutils_allocator_t)
{
  std::call_once(
    g_init_flag, [&]() {
      try {
        // 优先从环境变量读取配置文件路径，其次使用 config_file 参数
        const char * env_cfg = std::getenv("RCUTILS_LOGGING_CONFIG_FILE");
        std::string cfg_file;
        if (env_cfg && env_cfg[0] != '\0') {
          cfg_file = env_cfg;
        } else if (config_file && config_file[0] != '\0') {
          cfg_file = config_file;
        } else {
          cfg_file = "/tmp/ros2_spdlog.conf";
        }

        auto cfg = parse_config(cfg_file);

        std::string log_path =
        cfg.count("log_path") ? expand_tilde(cfg["log_path"]) : "/tmp/ros2.log";

        size_t max_size =
        cfg.count("max_size") ? std::stoull(cfg["max_size"]) : 10 * 1024 * 1024;

        size_t max_files =
        cfg.count("max_files") ? std::stoul(cfg["max_files"]) : 5;

        std::string pattern =
        cfg.count("pattern") ? cfg["pattern"] :
        "[%Y-%m-%d %H:%M:%S.%e] [%l] %v";

        std::string level =
        cfg.count("level") ? cfg["level"] : "info";

        size_t queue_size =
        cfg.count("queue_size") ? std::stoul(cfg["queue_size"]) : 8192;

        size_t thread_count =
        cfg.count("thread_count") ? std::stoul(cfg["thread_count"]) : 1;

        // ✅ 创建目录（C++17）
        std::filesystem::create_directories(
          std::filesystem::path(log_path).parent_path());

        // ✅ 初始化线程池
        spdlog::init_thread_pool(queue_size, thread_count);

        // ✅ 创建 rotating async logger
        g_logger = spdlog::rotating_logger_mt<spdlog::async_factory>(
          "ros2_rotating",
          log_path,
          max_size,
          max_files
        );

        g_logger->set_pattern(pattern);
        g_logger->set_level(str_to_level(level));
        g_logger->flush_on(spdlog::level::info);

        spdlog::set_default_logger(g_logger);

        fprintf(stderr, "🔥 rcl_logging_spdlog_rotating initialized\n");
      } catch (const std::exception & e) {
        fprintf(stderr, "[spdlog_rotating] EXCEPTION: %s\n", e.what());
      }
    });

  return RCL_LOGGING_RET_OK;
}

rcl_logging_ret_t rcl_logging_external_shutdown(void)
{
  if (g_logger) {
    g_logger->flush();
    spdlog::shutdown();
  }
  return RCL_LOGGING_RET_OK;
}

void rcl_logging_external_log(
  int severity,
  const char * name,
  const char * msg)
{
  if (!g_logger) {return;}

  const char * actual = extract_msg(name, msg);
  const char * n = name ? name : "";

  switch (severity) {
    case RCUTILS_LOG_SEVERITY_DEBUG:
      g_logger->debug("[{}]: {}", n, actual);
      break;
    case RCUTILS_LOG_SEVERITY_INFO:
      g_logger->info("[{}]: {}", n, actual);
      break;
    case RCUTILS_LOG_SEVERITY_WARN:
      g_logger->warn("[{}]: {}", n, actual);
      break;
    case RCUTILS_LOG_SEVERITY_ERROR:
      g_logger->error("[{}]: {}", n, actual);
      break;
    case RCUTILS_LOG_SEVERITY_FATAL:
      g_logger->critical("[{}]: {}", n, actual);
      break;
  }
}

rcl_logging_ret_t rcl_logging_external_set_logger_level(
  const char * name,
  int level)
{
  if (!g_logger) {return RCL_LOGGING_RET_ERROR;}
  (void)name;
  (void)level;
  return RCL_LOGGING_RET_OK;
}

}
