#include "voice_chat/app_config.hpp"
#include "voice_chat/voice_assistant.hpp"

#include <curl/curl.h>
#include <unitree/robot/channel/channel_factory.hpp>

#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <thread>
#include <utility>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void HandleSignal(int) { g_stop_requested = 1; }

class CurlGlobalGuard {
 public:
  CurlGlobalGuard() : initialized_(curl_global_init(CURL_GLOBAL_DEFAULT) == 0) {}
  ~CurlGlobalGuard() {
    if (initialized_) curl_global_cleanup();
  }

  bool IsInitialized() const { return initialized_; }

 private:
  bool initialized_{false};
};

}  // namespace

int main(int argc, char const *argv[]) {
  if (argc != 2) {
    std::cerr << "用法：g1_audio_client_test <G1有线网卡名>\n"
              << "例如：g1_audio_client_test enx00e04c316118" << std::endl;
    return 1;
  }

  std::string config_error;
  auto config = g1_voice::LoadAppConfig(config_error);
  if (!config) {
    std::cerr << "配置错误：" << config_error << std::endl;
    return 1;
  }

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  CurlGlobalGuard curl;
  if (!curl.IsInitialized()) {
    std::cerr << "libcurl全局初始化失败" << std::endl;
    return 1;
  }

  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    g1_voice::VoiceAssistant assistant(std::move(*config));

    std::string start_error;
    if (!assistant.Start(start_error)) {
      std::cerr << start_error << std::endl;
      return 1;
    }

    while (!g_stop_requested && assistant.IsRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    assistant.Stop();
  } catch (const std::exception &exception) {
    std::cerr << "程序异常：" << exception.what() << std::endl;
    return 1;
  }

  return 0;
}
