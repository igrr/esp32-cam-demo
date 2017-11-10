// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <sys/lock.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "rom/queue.h"

#include "esp_log.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"

#include "http_parser.h"
#include "http_server.h"


#define HTTP_URI_MAX_LEN 64

typedef enum {
    HTTP_PARSING_REQUEST,
    HTTP_COLLECTING_RESPONSE_HEADERS,
    HTTP_SENDING_RESPONSE_BODY,
    HTTP_DONE,
} http_state_t;

typedef struct http_header_t {
    char* name;
    char* value;
    SLIST_ENTRY(http_header_t) list_entry;
} http_header_t;

struct http_context_ {
    http_state_t state;
    char uri[HTTP_URI_MAX_LEN];
    struct netconn *conn;
    http_parser parser;
    int response_code;
    SLIST_HEAD(, http_header_t) response_headers;
    size_t expected_response_size;
    size_t accumulated_response_size;
};

typedef struct http_handler_t{
    char uri_pattern[HTTP_URI_MAX_LEN];
    int method;
    http_handler_fn_t cb;
    void* ctx;
    SLIST_ENTRY(http_handler_t) list_entry;
} http_handler_t;


struct http_server_context_ {
    int port;
    err_t server_task_err;
    struct netconn* server_conn;
    TaskHandle_t task;
    EventGroupHandle_t start_done;
    SLIST_HEAD(, http_handler_t) handlers;
    _lock_t handlers_lock;
    struct http_context_ connection_context;
};

#define SERVER_STARTED_BIT BIT(0)
#define SERVER_DONE_BIT BIT(1)


static const char* http_response_code_to_str(int code);

static const char* TAG = "http_server";

esp_err_t http_register_handler(http_server_t server,
        const char* uri_pattern, int method, http_handler_fn_t callback, void* callback_arg)
{
    http_handler_t* new_handler = (http_handler_t*) calloc(1, sizeof(*new_handler));
    if (new_handler == NULL) {
        return ESP_ERR_NO_MEM;
    }

    strlcpy(new_handler->uri_pattern, uri_pattern, sizeof(new_handler->uri_pattern));
    new_handler->cb = callback;
    new_handler->ctx = callback_arg;
    new_handler->method = method;

    _lock_acquire(&server->handlers_lock);
    /* FIXME: Handlers will be checked in the reverse order */
    SLIST_INSERT_HEAD(&server->handlers, new_handler, list_entry);
    _lock_release(&server->handlers_lock);
    return ESP_OK;
}

static http_handler_t* http_find_handler(http_server_t server, const char* uri, int method)
{
    http_handler_t* it;
    _lock_acquire(&server->handlers_lock);
    SLIST_FOREACH(it, &server->handlers, list_entry) {
        if (strcasecmp(uri, it->uri_pattern) == 0
            && method == it->method) {
            break;
        }
    }
    _lock_release(&server->handlers_lock);
    return it;
}

static int http_url_cb(http_parser* parser, const char *at, size_t length)
{
    http_context_t ctx = (http_context_t) parser->data;
    strncat(ctx->uri, at, MIN(length, HTTP_URI_MAX_LEN - 1));
    return 0;
}

static int http_headers_done_cb(http_parser* parser)
{
    http_context_t ctx = (http_context_t) parser->data;
    ctx->state = HTTP_COLLECTING_RESPONSE_HEADERS;
    return 0;
}


static esp_err_t lwip_err_to_esp_err(err_t e)
{
    switch (e) {
        case ERR_OK: return ESP_OK;
        case ERR_MEM: return ESP_ERR_NO_MEM;
        case ERR_TIMEOUT: return ESP_ERR_TIMEOUT;
        default:
            return ESP_FAIL;
    }
}

static void http_response_headers_clear(http_context_t http_ctx)
{
    http_header_t  *it, *next;
    SLIST_FOREACH_SAFE(it, &http_ctx->response_headers, list_entry, next) {
        SLIST_REMOVE(&http_ctx->response_headers, it, http_header_t, list_entry);
        free(it); /* frees memory allocated for header, name, and value */
    }
}

static esp_err_t http_add_content_length_header(http_context_t http_ctx, size_t value)
{
    char size_str[11];
    itoa(value, size_str, 10);
    return http_response_set_header(http_ctx, "Content-length", size_str);
}

static esp_err_t http_send_response_headers(http_context_t http_ctx)
{
    assert(http_ctx->state == HTTP_COLLECTING_RESPONSE_HEADERS);

    /* Calculate total size of all the headers, allocate a buffer */
    size_t total_headers_size = 0;

    /* response_code may be == 0, if we are sending headers for multipart
     * response part. In this case, don't send the response code line.
     */
    if (http_ctx->response_code > 0) {
        total_headers_size += 16 /* HTTP/1.1, code, CRLF */
                + strlen(http_response_code_to_str(http_ctx->response_code));
    }
    http_header_t* it;
    SLIST_FOREACH(it, &http_ctx->response_headers, list_entry) {
        total_headers_size += strlen(it->name) + strlen(it->value) + 4 /* ": ", CRLF */;
    }
    total_headers_size += 3; /* Final CRLF, '\0' terminator */
    char* headers_buf = calloc(1, total_headers_size);
    if (headers_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /* Write response */
    size_t buf_size = total_headers_size;
    char* buf_ptr = headers_buf;
    int len;
    if (http_ctx->response_code > 0) {
        len = snprintf(buf_ptr, buf_size, "HTTP/1.1 %d %s\r\n",
                http_ctx->response_code, http_response_code_to_str(http_ctx->response_code));
        assert(len < buf_size);
        buf_size -= len;
        buf_ptr += len;
    }

    /* Write response headers */
    SLIST_FOREACH(it, &http_ctx->response_headers, list_entry) {
        len = snprintf(buf_ptr, buf_size, "%s: %s\r\n", it->name, it->value);
        assert(len < buf_size);
        buf_size -= len;
        buf_ptr += len;
    }

    /* Final CRLF */
    len = snprintf(buf_ptr, buf_size, "\r\n");
    assert(len < buf_size);
    buf_size -= len;
    buf_ptr += len;

    http_response_headers_clear(http_ctx);

    err_t err = netconn_write(http_ctx->conn, headers_buf, strlen(headers_buf), NETCONN_COPY);
    free(headers_buf);

    http_ctx->state = HTTP_SENDING_RESPONSE_BODY;

    return lwip_err_to_esp_err(err);
}

/* Common function called by http_response_begin and http_response_begin_multipart */
static esp_err_t http_response_begin_common(http_context_t http_ctx, const char* content_type, size_t response_size)
{
    esp_err_t err = http_response_set_header(http_ctx, "Content-type", content_type);
    if (err != ESP_OK) {
        return err;
    }
    http_ctx->expected_response_size = response_size;
    http_ctx->accumulated_response_size = 0;
    if (response_size != HTTP_RESPONSE_SIZE_UNKNOWN) {
        err = http_add_content_length_header(http_ctx, response_size);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t http_response_begin(http_context_t http_ctx, int code, const char* content_type, size_t response_size)
{
    if (http_ctx->state != HTTP_COLLECTING_RESPONSE_HEADERS) {
        return ESP_ERR_INVALID_STATE;
    }
    http_ctx->response_code = code;
    return http_response_begin_common(http_ctx, content_type, response_size);
}

esp_err_t http_response_write(http_context_t http_ctx, const http_buffer_t* buffer)
{
    if (http_ctx->state == HTTP_COLLECTING_RESPONSE_HEADERS) {
        esp_err_t err = http_send_response_headers(http_ctx);
        if (err != ESP_OK) {
            return err;
        }
    }
    const int flag = buffer->data_is_persistent ? NETCONN_NOCOPY : NETCONN_COPY;
    size_t len = buffer->size ? buffer->size : strlen((const char*) buffer->data);
    err_t rc = netconn_write(http_ctx->conn, buffer->data, len, flag);
    if (rc != ESP_OK) {
        ESP_LOGD(TAG, "netconn_write rc=%d", rc);
    } else {
        http_ctx->accumulated_response_size += len;
    }
    return lwip_err_to_esp_err(rc);
}


esp_err_t http_response_end(http_context_t http_ctx)
{
    size_t expected = http_ctx->expected_response_size;
    size_t actual = http_ctx->accumulated_response_size;
    if (expected != HTTP_RESPONSE_SIZE_UNKNOWN && expected != actual) {
        ESP_LOGW(TAG, "Expected response size: %d, actual: %d", expected, actual);
    }
    http_ctx->state = HTTP_DONE;
    return ESP_OK;
}

esp_err_t http_response_begin_multipart(http_context_t http_ctx, const char* content_type, size_t response_size)
{
    if (http_ctx->state == HTTP_COLLECTING_RESPONSE_HEADERS) {
        http_send_response_headers(http_ctx);
        http_ctx->response_code = 0;
    }
    http_ctx->state = HTTP_COLLECTING_RESPONSE_HEADERS;
    return http_response_begin_common(http_ctx, content_type, response_size);
}

esp_err_t http_response_end_multipart(http_context_t http_ctx, const char* boundary)
{
    size_t expected = http_ctx->expected_response_size;
    size_t actual = http_ctx->accumulated_response_size;
    if (expected != HTTP_RESPONSE_SIZE_UNKNOWN && expected != actual) {
        ESP_LOGW(TAG, "Expected response size: %d, actual: %d", expected, actual);
    }
    /* reset expected_response_size so that http_response_end doesn't complain */
    http_ctx->expected_response_size = HTTP_RESPONSE_SIZE_UNKNOWN;

    const http_buffer_t buf = { .data = boundary };
    esp_err_t ret = http_response_write(http_ctx, &buf);
    http_ctx->state = HTTP_COLLECTING_RESPONSE_HEADERS;
    return ret;
}

esp_err_t http_response_set_header(http_context_t http_ctx, const char* name, const char* val)
{
    size_t name_len = strlen(name) + 1;
    size_t val_len = strlen(val) + 1;
    /* Allocate memory for the structure, name, and value, in one go */
    size_t buf_len = sizeof(http_header_t) + name_len + val_len;
    char* buf = (char*) calloc(1, buf_len);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    http_header_t* new_header = (http_header_t*) buf;
    new_header->name = buf + sizeof(http_header_t);
    new_header->value = new_header->name + name_len;
    strcpy(new_header->name, name);
    strcpy(new_header->value, val);
    SLIST_INSERT_HEAD(&http_ctx->response_headers, new_header, list_entry);
    return ESP_OK;
}


static void http_send_not_found_response(http_context_t http_ctx)
{
    http_response_begin(http_ctx, 404, "text/plain", HTTP_RESPONSE_SIZE_UNKNOWN);
    const http_buffer_t buf = {
            .data = "Not found",
            .data_is_persistent = true
    };
    http_response_write(http_ctx, &buf);
    http_response_end(http_ctx);
}


static const char* http_response_code_to_str(int code)
{
    switch (code) {
        case 200: return "OK";
        case 204: return "No Content";
        case 301: return "Moved Permanently";
        case 302: return "Found";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 405: return "Method Not Allowed";
        case 500: return "Internal Server Error";
        default:  return "";
      }
}


static void http_handle_connection(http_server_t server, struct netconn *conn)
{
    struct netbuf *inbuf = NULL;
    char *buf;
    u16_t buflen;
    err_t err = ERR_OK;

    /* Single threaded server, one context only */
    http_context_t ctx = &server->connection_context;

    /* Initialize context */
    ctx->state = HTTP_PARSING_REQUEST;
    memset(ctx->uri, 0, sizeof(ctx->uri));
    ctx->conn = conn;
    http_parser_init(&ctx->parser, HTTP_REQUEST);
    ctx->parser.data = ctx;

    const http_parser_settings parser_settings = {
            .on_url = &http_url_cb,
            .on_headers_complete = &http_headers_done_cb
    };

    while (ctx->state == HTTP_PARSING_REQUEST) {
        err = netconn_recv(conn, &inbuf);
        if (err != ERR_OK) {
            break;
        }

        err = netbuf_data(inbuf, (void**) &buf, &buflen);
        if (err != ERR_OK) {
            break;
        }

        size_t parsed_bytes = http_parser_execute(&ctx->parser, &parser_settings, buf, buflen);
        if (parsed_bytes < buflen) {
            break;
        }
    }

    if (err == ERR_OK && ctx->state == HTTP_COLLECTING_RESPONSE_HEADERS) {
        http_handler_t* handler = http_find_handler(server, ctx->uri, (int) ctx->parser.method);
        if (handler == NULL) {
            http_send_not_found_response(ctx);
        } else {
            (*handler->cb)(ctx, handler->ctx);
        }
    }
    if (err != ERR_CLSD) {
        netconn_close(conn);
    }
    if (inbuf) {
        netbuf_delete(inbuf);
    }
}


static void http_server(void *arg)
{
    http_server_t ctx = (http_server_t) arg;
    struct netconn *client_conn;
    err_t err;
    ctx->server_conn = netconn_new(NETCONN_TCP);
    if (ctx->server_conn == NULL) {
        err = ERR_MEM;
        goto out;
    }

    err = netconn_bind(ctx->server_conn, NULL, ctx->port);
    if (err != ERR_OK) {
        goto out;
    }

    err = netconn_listen(ctx->server_conn);
    if (err != ERR_OK) {
        goto out;
    }
    xEventGroupSetBits(ctx->start_done, SERVER_STARTED_BIT);

    do {
        err = netconn_accept(ctx->server_conn, &client_conn);
        if (err == ERR_OK) {
            http_handle_connection(ctx, client_conn);
            netconn_delete(client_conn);
        }
    } while (err == ERR_OK);

out:
    if (ctx->server_conn) {
        netconn_close(ctx->server_conn);
        netconn_delete(ctx->server_conn);
    }
    if (err != ERR_OK) {
        ctx->server_task_err = err;
        xEventGroupSetBits(ctx->start_done, SERVER_DONE_BIT);
    }
    vTaskDelete(NULL);
}

esp_err_t http_server_start(int port, http_server_t* out_server)
{
    http_server_t ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ctx->port = port;
    ctx->start_done = xEventGroupCreate();
    if (ctx->start_done == NULL) {
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    int ret = xTaskCreatePinnedToCore(&http_server, "httpd", 4096, ctx, 5, &ctx->task, 0);
    if (ret != pdPASS) {
        vEventGroupDelete(ctx->start_done);
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    int bits = xEventGroupWaitBits(ctx->start_done, SERVER_STARTED_BIT | SERVER_DONE_BIT, 0, 0, portMAX_DELAY);
    if (bits & SERVER_DONE_BIT) {
        /* Error happened, task is deleted */
        esp_err_t err = lwip_err_to_esp_err(ctx->server_task_err);
        vEventGroupDelete(ctx->start_done);
        free(ctx);
        return err;
    }

    *out_server = ctx;
    return ESP_OK;
}

esp_err_t http_server_stop(http_server_t server)
{
    /* FIXME: figure out a thread safe way to do this */
    netconn_close(server->server_conn);
    xEventGroupWaitBits(server->start_done, SERVER_DONE_BIT, 0, 0, portMAX_DELAY);
    free(server);
    return ESP_OK;
}
