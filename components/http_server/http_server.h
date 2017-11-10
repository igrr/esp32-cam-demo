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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @file http_server.h
 * @brief Minimal HTTP server
 */

/* Pull in the definitions of HTTP methods */
#include "http_parser.h"

/** Opaque type representing single HTTP connection */
typedef struct http_context_* http_context_t;

/** Opaque type representing HTTP server */
typedef struct http_server_context_* http_server_t;

/** Callback type of HTTP request handler */
typedef void (* http_handler_fn_t)(http_context_t http_ctx, void* ctx);

/**
 * @brief initialize HTTP server, start listening
 * @param port  port number to listen on
 * @param[out] output, handle of the server; pass it to http_server_stop do
 *             delete the server.
 * @return
 *  - ESP_OK on success
 *  - ESP_ERR_NO_MEM if out of RAM
 *  - ESP_FAIL if some other error
 */
esp_err_t http_server_start(int port, http_server_t* out_server);

/**
 * @brief Stop the previously started server
 * @param server handle obtained from http_server_start
 * @return
 *  - ESP_OK on success
 */
esp_err_t http_server_stop(http_server_t server);

/**
 * @brief Register a handler for certain URI
 *
 * The handler will be called when a client makes a request with matching URI
 * and HTTP method.
 *
 * @note Currently only matches full URIs, doesn't support regex
 *
 * @note Request body is currently ignored
 *
 * @param uri_pattern URI pattern to match
 * @param method one of HTTP_GET, HTTP_POST, HTTP_PUT, etc
 * @param callback function to call
 * @param callback_arg application context to pass to the callback
 * @return
 *  - ESP_OK on success
 *  - ESP_ERR_NO_MEM if out of memory
 */
esp_err_t http_register_handler(http_server_t server, const char* uri_pattern, int method,
                                http_handler_fn_t callback, void* callback_arg);

/**
 * @brief structure describing a part of the response to be sent
 */
typedef struct {
    const void* data;     /*!< pointer to data to be sent */
    size_t size;    /*!< size of data to send; if data is a 0-terminated string, size can be left 0 */
    bool data_is_persistent;    /*!< set to true if data is in constant RAM */
} http_buffer_t;

#define HTTP_RESPONSE_SIZE_UNKNOWN SIZE_MAX

/**
 * @brief Begin writing HTTP response
 * @param http_ctx  context passed to the handler
 * @param code  HTTP response code
 * @param content_type  string to send as a value in content-type header
 * @param response_size  either the size of the response body, or HTTP_RESPONSE_SIZE_UNKNOWN
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NO_MEM if can't allocate temporary buffer
 *      - other errors from LwIP
 */
esp_err_t http_response_begin(http_context_t http_ctx, int code,
                              const char* content_type, size_t response_size);

/**
 * @brief Add HTTP header to the response
 *
 * This function may be called between http_response_begin and http_response_write.
 *
 * For multipart responses, calling it between http_response_begin and
 * http_response_begin_multipart adds header to the list of headers in the original
 * response (which is sent first). Calling it after http_response_begin_multipart
 * adds the header to the list of headers sent for the current response part.
 *
 * 'name' and 'val' can point to temporary values. This function copies the
 * both strings into internal storage.
 *
 * @param http_ctx  context passed to the handler
 * @param name  Header name
 * @param val   Header value
 * @return
 *  - ESP_OK on success
 *  - ESP_ERR_NO_MEM if can't allocate memory for the header
 */
esp_err_t http_response_set_header(http_context_t http_ctx,
                                   const char* name, const char* val);

/**
 * @brief Start one part of a multipart response
 *
 * For multipart responses, the sequence of calls is as follows:
 *
 * 1. http_response_begin (with size == HTTP_RESPONSE_SIZE_UNKNOWN)
 * 2. (optional) http_response_set_header — for the overall response
 * 3. http_response_begin_multipart (if part size is known, pass it in response_size argument)
 * 4. (optional) http_repponse_set_header — for the current part
 * 5. (optional) http_response_write — write response data
 * 6. http_response_end_multipart — when done with the part
 * 7. repeat from 3 as needed
 *
 * @param http_ctx context passed to the handler
 * @param content_type  value to be set in part's content-type header
 * @param response_size  either the size of part body, or HTTP_RESPONSE_SIZE_UNKNOWN
 * @return
 *  - ESP_OK
 *  - ESP_ERR_NO_MEM if can't allocate memory
 *  - other errors from LwIP
 */
esp_err_t http_response_begin_multipart(http_context_t http_ctx,
                                const char* content_type, size_t response_size);

/**
 * @brief Indicate that one part of the multipart response is finished
 * @param http_ctx  context passed to the handler
 * @param boundary  part boundary. Has to be the same as given in the content-type of the first response.
 * @return
 *  - ESP_OK
 *  - other errors from LwIP
 */
esp_err_t http_response_end_multipart(http_context_t http_ctx, const char* boundary);

/**
 * @brief Send a piece of HTTP response to the client
 * @param http_ctx  context passed to the handler
 * @param buffer  data to send, see \ref http_buffer_t
 * @return
 *      - ESP_OK on success
 *      - other errors from LwIP
 */
esp_err_t http_response_write(http_context_t http_ctx, const http_buffer_t* buffer);

/**
 * @brief Indicate that response is complete
 * @param http_ctx  context passed to the handler
 * @return
 *      - ESP_OK on success
 *      - other errors in the future?
 */
esp_err_t http_response_end(http_context_t http_ctx);

#ifdef __cplusplus
}
#endif
