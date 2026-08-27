#include "voice_chat/llm_client.hpp"

#include "voice_chat/asr_processor.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <array>
#include <memory>
#include <utility>

namespace g1_voice {
namespace {

using Json = nlohmann::json;
constexpr std::size_t kMaximumResponseBytes = 1024 * 1024;

struct CurlDeleter {
  void operator()(CURL *curl) const {
    if (curl != nullptr) curl_easy_cleanup(curl);
  }
};

struct HeaderDeleter {
  void operator()(curl_slist *headers) const {
    if (headers != nullptr) curl_slist_free_all(headers);
  }
};

struct ResponseBuffer {
  std::string body;
  bool exceeded_limit{false};
};

size_t CurlWriteCallback(char *data, size_t size, size_t count,
                         void *user_data) {
  const size_t bytes = size * count;
  auto *buffer = static_cast<ResponseBuffer *>(user_data);
  if (buffer->body.size() + bytes > kMaximumResponseBytes) {
    buffer->exceeded_limit = true;
    return 0;
  }
  buffer->body.append(data, bytes);
  return bytes;
}

int CurlProgressCallback(void *user_data, curl_off_t, curl_off_t, curl_off_t,
                         curl_off_t) {
  const auto *running = static_cast<const std::atomic<bool> *>(user_data);
  return running->load() ? 0 : 1;
}

bool AppendHeader(curl_slist *&headers, const std::string &header) {
  curl_slist *updated = curl_slist_append(headers, header.c_str());
  if (updated == nullptr) return false;
  headers = updated;
  return true;
}

std::string ExtractHttpError(const std::string &body) {
  try {
    const Json response = Json::parse(body);
    if (response.contains("error") && response["error"].is_object()) {
      const Json &api_error = response["error"];
      if (api_error.contains("message") && api_error["message"].is_string()) {
        return api_error["message"].get<std::string>();
      }
    }
    if (response.contains("base_resp") && response["base_resp"].is_object()) {
      const Json &base_response = response["base_resp"];
      if (base_response.contains("status_msg") &&
          base_response["status_msg"].is_string()) {
        return base_response["status_msg"].get<std::string>();
      }
    }
  } catch (const Json::exception &) {
  }

  return body.substr(0, 300);
}

}  // namespace

LlmClient::LlmClient(LlmConfig config, const std::atomic<bool> &running)
    : config_(std::move(config)), running_(running) {}

std::optional<std::string> LlmClient::Ask(const std::string &question,
                                          std::string &error) {
  Json messages = Json::array();
  messages.push_back({
      {"role", "system"},
      {"content",
       "你是运行在宇树G1机器人上的中文语音助手。请自然、友好、简洁地回答。"
       "每次回答一到三句话，尽量不超过八十个汉字；不要使用Markdown、表格或动作指令。"
       "直接给出最终回答，不要展示思考过程。"}});

  for (const auto &message : history_) {
    messages.push_back(
        {{"role", message.role}, {"content", message.content}});
  }
  messages.push_back({{"role", "user"}, {"content", question}});

  Json request = {{"model", config_.model},
                  {"messages", messages},
                  {"temperature", config_.temperature},
                  {"max_tokens", config_.max_tokens},
                  {"stream", false}};
  if (config_.reasoning_split) request["reasoning_split"] = true;

  std::unique_ptr<CURL, CurlDeleter> curl(curl_easy_init());
  if (!curl) {
    error = "无法初始化 libcurl";
    return std::nullopt;
  }

  curl_slist *raw_headers = nullptr;
  const std::string authorization =
      "Authorization: Bearer " + config_.api_key;
  if (!AppendHeader(raw_headers, authorization) ||
      !AppendHeader(raw_headers, "Content-Type: application/json")) {
    if (raw_headers != nullptr) curl_slist_free_all(raw_headers);
    error = "无法创建大模型请求头";
    return std::nullopt;
  }
  std::unique_ptr<curl_slist, HeaderDeleter> headers(raw_headers);

  ResponseBuffer response;
  std::array<char, CURL_ERROR_SIZE> curl_error{};
  const std::string request_body = request.dump();

  curl_easy_setopt(curl.get(), CURLOPT_URL, config_.api_url.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_POST, 1L);
  curl_easy_setopt(curl.get(), CURLOPT_HTTPHEADER, headers.get());
  curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDS, request_body.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDSIZE,
                   static_cast<long>(request_body.size()));
  curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, CurlWriteCallback);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl.get(), CURLOPT_CONNECTTIMEOUT,
                   config_.connect_timeout_seconds);
  curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT, config_.request_timeout_seconds);
  curl_easy_setopt(curl.get(), CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl.get(), CURLOPT_TCP_KEEPALIVE, 1L);
  curl_easy_setopt(curl.get(), CURLOPT_ACCEPT_ENCODING, "");
  curl_easy_setopt(curl.get(), CURLOPT_USERAGENT,
                   "unitree-g1-voice-assistant/2.0");
  curl_easy_setopt(curl.get(), CURLOPT_ERRORBUFFER, curl_error.data());
  curl_easy_setopt(curl.get(), CURLOPT_NOPROGRESS, 0L);
  curl_easy_setopt(curl.get(), CURLOPT_XFERINFOFUNCTION, CurlProgressCallback);
  curl_easy_setopt(curl.get(), CURLOPT_XFERINFODATA, &running_);

  const CURLcode result = curl_easy_perform(curl.get());
  long http_status = 0;
  curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &http_status);

  if (response.exceeded_limit) {
    error = "大模型响应超过1MB，已停止接收";
    return std::nullopt;
  }
  if (result != CURLE_OK) {
    if (result == CURLE_ABORTED_BY_CALLBACK && !running_.load()) {
      error = "大模型请求已取消";
    } else if (curl_error[0] != '\0') {
      error = std::string("大模型网络请求失败：") + curl_error.data();
    } else {
      error = std::string("大模型网络请求失败：") +
              curl_easy_strerror(result);
    }
    return std::nullopt;
  }
  if (http_status < 200 || http_status >= 300) {
    error = "大模型接口返回 HTTP " + std::to_string(http_status);
    const std::string detail = ExtractHttpError(response.body);
    if (!detail.empty()) error += "：" + detail;
    return std::nullopt;
  }

  try {
    const Json parsed = Json::parse(response.body);
    if (!parsed.contains("choices") || !parsed["choices"].is_array() ||
        parsed["choices"].empty()) {
      error = "大模型响应中缺少 choices";
      return std::nullopt;
    }

    const Json &message = parsed["choices"][0]["message"];
    if (!message.contains("content") || !message["content"].is_string()) {
      error = "大模型响应中缺少文字内容";
      return std::nullopt;
    }

    const std::string answer = Trim(message["content"].get<std::string>());
    if (answer.empty()) {
      error = "大模型返回了空回答";
      return std::nullopt;
    }

    history_.push_back({"user", question});
    history_.push_back({"assistant", answer});
    while (history_.size() > 12) history_.pop_front();
    return answer;
  } catch (const Json::exception &exception) {
    error = std::string("无法解析大模型返回内容：") + exception.what();
    return std::nullopt;
  }
}

void LlmClient::ClearHistory() { history_.clear(); }

}  // namespace g1_voice
