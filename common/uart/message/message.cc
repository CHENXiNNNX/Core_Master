/**
 * @file    message.cc
 * @brief   UART消息序列化实现
 */

#include "message.hpp"
#include <cstdio>
#include <cstring>

namespace app::common::uart::message
{

                /*============================================================================
                 * JSON 工具实现
                 *============================================================================*/

                std::string json_escape(const std::string& s)
                {
                    std::string out;
                    out.reserve(s.size() + 16);
                    for (char c : s)
                    {
                        switch (c)
                        {
                        case '"':
                            out += "\\\"";
                            break;
                        case '\\':
                            out += "\\\\";
                            break;
                        case '\n':
                            out += "\\n";
                            break;
                        case '\r':
                            out += "\\r";
                            break;
                        case '\t':
                            out += "\\t";
                            break;
                        default:
                            out += c;
                            break;
                        }
                    }
                    return out;
                }

                bool json_get_str(const std::string& json, const char* key, std::string& out)
                {
                    std::string pattern = std::string("\"") + key + "\":\"";
                    size_t      pos     = json.find(pattern);
                    if (pos == std::string::npos)
                        return false;

                    pos += pattern.length();
                    std::string result;
                    bool        escaped = false;

                    for (size_t i = pos; i < json.size(); ++i)
                    {
                        char c = json[i];
                        if (escaped)
                        {
                            switch (c)
                            {
                            case 'n':
                                result += '\n';
                                break;
                            case 'r':
                                result += '\r';
                                break;
                            case 't':
                                result += '\t';
                                break;
                            case '"':
                                result += '"';
                                break;
                            case '\\':
                                result += '\\';
                                break;
                            default:
                                result += c;
                                break;
                            }
                            escaped = false;
                        }
                        else if (c == '\\')
                        {
                            escaped = true;
                        }
                        else if (c == '"')
                        {
                            out = result;
                            return true;
                        }
                        else
                        {
                            result += c;
                        }
                    }
                    return false;
                }

                bool json_get_int(const std::string& json, const char* key, int32_t& out)
                {
                    std::string pattern = std::string("\"") + key + "\":";
                    size_t      pos     = json.find(pattern);
                    if (pos == std::string::npos)
                        return false;

                    pos += pattern.length();
                    while (pos < json.size() && json[pos] == ' ')
                        pos++;

                    if (pos >= json.size())
                        return false;

                    char* end = nullptr;
                    long  val = strtol(json.c_str() + pos, &end, 10);
                    if (end == json.c_str() + pos)
                        return false;

                    out = static_cast<int32_t>(val);
                    return true;
                }

                bool json_get_bool(const std::string& json, const char* key, bool& out)
                {
                    std::string pattern = std::string("\"") + key + "\":";
                    size_t      pos     = json.find(pattern);
                    if (pos == std::string::npos)
                        return false;

                    pos += pattern.length();
                    while (pos < json.size() && json[pos] == ' ')
                        pos++;

                    if (json.compare(pos, 4, "true") == 0)
                    {
                        out = true;
                        return true;
                    }
                    if (json.compare(pos, 5, "false") == 0)
                    {
                        out = false;
                        return true;
                    }
                    return false;
                }

                bool json_get_obj(const std::string& json, const char* key, std::string& out)
                {
                    std::string pattern = std::string("\"") + key + "\":";
                    size_t      pos     = json.find(pattern);
                    if (pos == std::string::npos)
                        return false;

                    pos += pattern.length();
                    while (pos < json.size() && json[pos] == ' ')
                        pos++;

                    if (pos >= json.size())
                        return false;

                    char start = json[pos];
                    char end_char;
                    if (start == '{')
                        end_char = '}';
                    else if (start == '[')
                        end_char = ']';
                    else
                        return false;

                    int    depth     = 1;
                    size_t start_pos = pos;
                    pos++;

                    while (pos < json.size() && depth > 0)
                    {
                        char c = json[pos];
                        if (c == start)
                            depth++;
                        else if (c == end_char)
                            depth--;
                        else if (c == '"')
                        {
                            pos++;
                            while (pos < json.size())
                            {
                                if (json[pos] == '\\')
                                    pos += 2;
                                else if (json[pos] == '"')
                                {
                                    pos++;
                                    break;
                                }
                                else
                                    pos++;
                            }
                            continue;
                        }
                        pos++;
                    }

                    if (depth == 0)
                    {
                        out = json.substr(start_pos, pos - start_pos);
                        return true;
                    }
                    return false;
                }

                /*============================================================================
                 * Message 实现
                 *============================================================================*/

                std::string Message::to_json() const
                {
                    std::string json = "{";

                    // mod
                    json += "\"mod\":\"" + json_escape(mod) + "\"";

                    // cmd
                    json += ",\"cmd\":\"" + json_escape(cmd) + "\"";

                    // id（仅非0时输出）
                    if (id != 0)
                    {
                        char buf[16];
                        snprintf(buf, sizeof(buf), "%d", id);
                        json += ",\"id\":";
                        json += buf;
                    }

                    // ok（仅false时输出）
                    if (!ok)
                    {
                        json += ",\"ok\":false";
                    }

                    // err（仅非空时输出）
                    if (!err.empty())
                    {
                        json += ",\"err\":\"" + json_escape(err) + "\"";
                    }

                    // data（仅非空时输出，已是JSON格式）
                    if (!data.empty())
                    {
                        json += ",\"data\":";
                        json += data;
                    }

                    json += "}";
                    return json;
                }

                bool Message::from_json(const std::string& json, Message& msg)
                {
                    msg = Message();

                    // mod（必需）
                    if (!json_get_str(json, "mod", msg.mod))
                        return false;

                    // cmd（必需）
                    if (!json_get_str(json, "cmd", msg.cmd))
                        return false;

                    // id（可选）
                    json_get_int(json, "id", msg.id);

                    // ok（可选，默认true）
                    msg.ok = true;
                    json_get_bool(json, "ok", msg.ok);

                    // err（可选）
                    json_get_str(json, "err", msg.err);

                    // data（可选，提取为原始JSON）
                    json_get_obj(json, "data", msg.data);

                    return true;
                }

                Message Message::reply_ok(const std::string& resp_data) const
                {
                    Message reply;
                    reply.mod  = mod;
                    reply.cmd  = cmd;
                    reply.id   = id;
                    reply.ok   = true;
                    reply.data = resp_data;
                    return reply;
                }

                Message Message::reply_err(const std::string& error) const
                {
                    Message reply;
                    reply.mod = mod;
                    reply.cmd = cmd;
                    reply.id  = id;
                    reply.ok  = false;
                    reply.err = error;
                    return reply;
        }

} // namespace app::common::uart::message
