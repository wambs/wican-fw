/*
 * Copyright (c) 2022 Tomoyuki Sakurai <y@trombik.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this
 *  list of conditions and the following disclaimer in the documentation and/or
 *  other materials provided with the distribution.
 *
 * 3. Neither the name of "Floorsense Ltd", "Agile Workspace Ltd" nor the names of
 *  its contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <assert.h>
#include <string.h>
#include <inttypes.h>
#include <lwip/ip.h>
#include <lwip/netdb.h>
#include <lwip/err.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <lwip/netif.h>
#include <esp_wireguard.h>
#include <mbedtls/base64.h>

#include "wireguard-platform.h"
#include "wireguardif.h"

#define TAG "esp_wireguard"
#define WG_KEY_LEN  (32)
#define WG_B64_KEY_LEN (4 * ((WG_KEY_LEN + 2) / 3))
#if defined(CONFIG_LWIP_IPV6)
#define WG_ADDRSTRLEN  INET6_ADDRSTRLEN
#else
#define WG_ADDRSTRLEN  INET_ADDRSTRLEN
#endif

static struct netif wg_netif_struct = {0};
static struct netif *wg_netif = NULL;
static struct wireguardif_peer peer = {0};
static uint8_t wg_local_peer_idx = WIREGUARDIF_INVALID_INDEX;
static uint8_t preshared_key_decoded[WG_KEY_LEN];

static esp_err_t esp_wireguard_peer_init(const wireguard_config_t *config, struct wireguardif_peer *peer)
{
    esp_err_t err;
    char addr_str[WG_ADDRSTRLEN];
    struct addrinfo *res = NULL;
    struct addrinfo hints;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;

    if (!config || !peer) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    peer->public_key = config->public_key;
    if (config->preshared_key != NULL) {
        size_t len;
        int res;

        ESP_LOGI(TAG, "using preshared_key");
        ESP_LOGD(TAG, "preshared_key: %s", config->preshared_key);
#if defined(CONFIG_WIREGUARD_x25519_IMPLEMENTATION_DEFAULT)
        ESP_LOGI(TAG, "X25519: default");
#elif defined(CONFIG_WIREGUARD_x25519_IMPLEMENTATION_NACL)
        ESP_LOGI(TAG, "X25519: NaCL");
#endif
        res = mbedtls_base64_decode(preshared_key_decoded, WG_KEY_LEN, &len, (unsigned char *)config->preshared_key, WG_B64_KEY_LEN);
        if (res != 0 || len != WG_KEY_LEN) {
            err = ESP_FAIL;
            ESP_LOGE(TAG, "base64_decode: %i", res);
            if (len != WG_KEY_LEN) {
                ESP_LOGE(TAG, "invalid decoded length, len: %u, should be %u", len, WG_KEY_LEN);
            }
            goto fail;
        }
        peer->preshared_key = preshared_key_decoded;
    } else {
        peer->preshared_key = NULL;
    }
    peer->keep_alive = config->persistent_keepalive;

    /* Allow all IPs through tunnel */
    {
        ip_addr_t allowed_ip = IPADDR4_INIT_BYTES(0, 0, 0, 0);
        peer->allowed_ip = allowed_ip;
        ip_addr_t allowed_mask = IPADDR4_INIT_BYTES(0, 0, 0, 0);
        peer->allowed_mask = allowed_mask;
    }

    /* resolve peer name or IP address */
    {
        ip_addr_t endpoint_ip;
        memset(&endpoint_ip, 0, sizeof(endpoint_ip));

        /* XXX lwip_getaddrinfo returns only the first address of a host at the moment */
        if (getaddrinfo(config->endpoint, NULL, &hints, &res) != 0) {
            err = ESP_FAIL;

            /* XXX gai_strerror() is not implemented */
            ESP_LOGE(TAG, "getaddrinfo: unable to resolve `%s`", config->endpoint);
            goto fail;
        }

        if (res->ai_family == AF_INET) {
            struct in_addr addr4 = ((struct sockaddr_in *) (res->ai_addr))->sin_addr;
            inet_addr_to_ip4addr(ip_2_ip4(&endpoint_ip), &addr4);
        } else {
#if defined(CONFIG_LWIP_IPV6)
            struct in6_addr addr6 = ((struct sockaddr_in6 *) (res->ai_addr))->sin6_addr;
            inet6_addr_to_ip6addr(ip_2_ip6(&endpoint_ip), &addr6);
#endif
        }
        peer->endpoint_ip = endpoint_ip;

        if (inet_ntop(res->ai_family, &(peer->endpoint_ip), addr_str, WG_ADDRSTRLEN) == NULL) {
            ESP_LOGW(TAG, "inet_ntop: %i", errno);
        } else {
            ESP_LOGI(TAG, "Peer: %s (%s:%i)",
                                            config->endpoint,
                                            addr_str,
                                            config->port);
        }
    }
    peer->endport_port = config->port;
    peer->keep_alive = config->persistent_keepalive;
    err = ESP_OK;
fail:
    freeaddrinfo(res);
    return err;
}

static esp_err_t esp_wireguard_netif_create(const wireguard_config_t *config)
{
    esp_err_t err;
    ip_addr_t ip_addr;
    ip_addr_t netmask;
    ip_addr_t gateway = IPADDR4_INIT_BYTES(0, 0, 0, 0);
    struct wireguardif_init_data wg = {0};

    if (!config) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    /* Setup the WireGuard device structure */
    wg.private_key = config->private_key;
    wg.listen_port = config->listen_port;

    /* Restore v1.06 behavior: pin the WG UDP socket to the underlying WiFi
     * STA netif. Without this, once the WG netif becomes the default route,
     * the encrypted tunnel packets themselves are routed back into the WG
     * netif (encrypt -> route -> encrypt ...), recursing until the TCPIP
     * thread stack overflows and the device panics. */
    {
        struct netif *underlying_netif = NULL;
        char lwip_netif_name[8] = {0,};
        if (esp_netif_get_netif_impl_name(
                esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),
                lwip_netif_name) == ESP_OK) {
            underlying_netif = netif_find(lwip_netif_name);
        }
        if (underlying_netif == NULL) {
            ESP_LOGE(TAG, "cannot resolve WIFI_STA_DEF netif for WireGuard bind");
            err = ESP_FAIL;
            goto fail;
        }
        wg.bind_netif = underlying_netif;
    }

    ESP_LOGI(TAG, "allowed_ip: %s", config->allowed_ip);

    if (ipaddr_aton(config->allowed_ip, &ip_addr) != 1) {
        ESP_LOGE(TAG, "ipaddr_aton: invalid allowed_ip: `%s`", config->allowed_ip);
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    if (ipaddr_aton(config->allowed_ip_mask, &netmask) != 1) {
        ESP_LOGE(TAG, "ipaddr_aton: invalid allowed_ip_mask: `%s`", config->allowed_ip_mask);
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    /* Register the new WireGuard network interface with lwIP */
    wg_netif = netif_add(
            &wg_netif_struct,
            ip_2_ip4(&ip_addr),
            ip_2_ip4(&netmask),
            ip_2_ip4(&gateway),
            &wg, &wireguardif_init,
            &ip_input);
    if (wg_netif == NULL) {
        ESP_LOGE(TAG, "netif_add: failed");
        err = ESP_FAIL;
        goto fail;
    }

    /* Mark the interface as administratively up, link up flag is set
     * automatically when peer connects */
    netif_set_up(wg_netif);
    err = ESP_OK;
fail:
    return err;
}

esp_err_t esp_wireguard_init(wireguard_config_t *config, wireguard_ctx_t *ctx)
{
    esp_err_t err = ESP_FAIL;

    if (!config || !ctx) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    err = wireguard_platform_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wireguard_platform_init: %s", esp_err_to_name(err));
        goto fail;
    }
    ctx->config = config;
    ctx->netif = NULL;
    ctx->netif_default = netif_default;

    err = ESP_OK;
fail:
    return err;
}

/*
 * Threading fix: everything below that touches lwIP structures directly
 * (netif_add/remove, netif_set_default, the UDP PCB and sys_timeout work
 * done inside wireguardif_init, and WireGuard peer/keypair state shared
 * with the tcpip thread) must execute in tcpip thread context. This
 * project builds with CONFIG_LWIP_TCPIP_CORE_LOCKING disabled, so calling
 * these directly from the vpn_task races the tcpip thread and can corrupt
 * lwIP state (observed as panics during WireGuard (re)connect shortly
 * after wake). esp_netif_tcpip_exec() runs a callback synchronously in
 * tcpip context, which makes the whole chain safe — including everything
 * netif_add() calls internally.
 *
 * Blocking calls (getaddrinfo inside esp_wireguard_peer_init) must stay
 * OUTSIDE these callbacks: they need the tcpip thread to make progress
 * and would deadlock if run inside it. That is why peer_init now runs
 * first, in the calling task, before the marshaled section.
 */

static esp_err_t wg_connect_in_tcpip_ctx(void *arg)
{
    wireguard_ctx_t *ctx = (wireguard_ctx_t *)arg;
    esp_err_t err = ESP_FAIL;
    err_t lwip_err = -1;
    bool created_netif = false;

    if (ctx->netif == NULL) {
        err = esp_wireguard_netif_create(ctx->config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wireguard_netif_create: %s", esp_err_to_name(err));
            goto fail;
        }
        created_netif = true;

        /* Register the new WireGuard peer with the network interface.
         * (peer was already initialized by esp_wireguard_peer_init in the
         * calling task — see esp_wireguard_connect below.) */
        lwip_err = wireguardif_add_peer(wg_netif, &peer, &wg_local_peer_idx);
        if (lwip_err != ERR_OK || wg_local_peer_idx == WIREGUARDIF_INVALID_INDEX) {
            ESP_LOGE(TAG, "wireguardif_add_peer: %i", lwip_err);
            /* NOTE: the pre-fix code fell through here with err still ESP_OK
             * (set by the earlier steps), silently reporting success and
             * skipping cleanup on add_peer failure. Set it explicitly. */
            err = ESP_FAIL;
            goto fail;
        }
        if (ip_addr_isany(&peer.endpoint_ip)) {
            err = ESP_FAIL;
            goto fail;
        }
        ctx->netif = wg_netif;
        ctx->netif_default = netif_default;
    }

    ESP_LOGI(TAG, "Connecting to %s:%i", ctx->config->endpoint, ctx->config->port);
    lwip_err = wireguardif_connect(wg_netif, wg_local_peer_idx);
    if (lwip_err != ERR_OK) {
        ESP_LOGE(TAG, "wireguardif_connect: %i", lwip_err);
        err = ESP_FAIL;
        goto fail;
    }
    err = ESP_OK;
fail:
    // If we created the netif in this call but failed before ctx->netif was set,
    // clean up the lwIP netif. Otherwise a later retry will hit:
    // assert failed: netif_add (... netif already added)
    if (err != ESP_OK && created_netif && ctx->netif == NULL && wg_netif != NULL)
    {
        // Best-effort cleanup (some steps may fail depending on how far we got)
        if (wg_local_peer_idx != WIREGUARDIF_INVALID_INDEX)
        {
            (void)wireguardif_disconnect(wg_netif, wg_local_peer_idx);
            (void)wireguardif_remove_peer(wg_netif, wg_local_peer_idx);
            wg_local_peer_idx = WIREGUARDIF_INVALID_INDEX;
        }
        wireguardif_shutdown(wg_netif);
        netif_remove(wg_netif);
        wireguardif_fini(wg_netif);
        wg_netif = NULL;
        memset(&wg_netif_struct, 0, sizeof(wg_netif_struct));
    }
    return err;
}

esp_err_t esp_wireguard_connect(wireguard_ctx_t *ctx)
{
    esp_err_t err;

    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ctx->netif == NULL) {
        /* Initialize the first WireGuard peer structure. Does blocking DNS
         * (getaddrinfo) — must run here in the calling task, NOT inside
         * the tcpip context below (would deadlock). */
        err = esp_wireguard_peer_init(ctx->config, &peer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wireguard_peer_init: %s", esp_err_to_name(err));
            return err;
        }
    }

    return esp_netif_tcpip_exec(wg_connect_in_tcpip_ctx, ctx);
}

static esp_err_t wg_set_default_in_tcpip_ctx(void *arg)
{
    wireguard_ctx_t *ctx = (wireguard_ctx_t *)arg;
    netif_set_default(ctx->netif);
    return ESP_OK;
}

esp_err_t esp_wireguard_set_default(wireguard_ctx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_netif_tcpip_exec(wg_set_default_in_tcpip_ctx, ctx);
}

static esp_err_t wg_disconnect_in_tcpip_ctx(void *arg)
{
    wireguard_ctx_t *ctx = (wireguard_ctx_t *)arg;
    err_t lwip_err;

    // Clear the IP address to gracefully disconnect any clients while the
    // peers are still valid
    netif_set_ipaddr(ctx->netif, IP4_ADDR_ANY4);

    lwip_err = wireguardif_disconnect(ctx->netif, wg_local_peer_idx);
    if (lwip_err != ERR_OK) {
        ESP_LOGW(TAG, "wireguardif_disconnect: peer_index: %" PRIu8 " err: %i", wg_local_peer_idx, lwip_err);
    }

    lwip_err = wireguardif_remove_peer(ctx->netif, wg_local_peer_idx);
    if (lwip_err != ERR_OK) {
        ESP_LOGW(TAG, "wireguardif_remove_peer: peer_index: %" PRIu8 " err: %i", wg_local_peer_idx, lwip_err);
    }

    wg_local_peer_idx = WIREGUARDIF_INVALID_INDEX;
    wireguardif_shutdown(ctx->netif);
    netif_remove(ctx->netif);
    wireguardif_fini(ctx->netif);
    netif_set_default(ctx->netif_default);
    ctx->netif = NULL;

    return ESP_OK;
}

esp_err_t esp_wireguard_disconnect(wireguard_ctx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_netif_tcpip_exec(wg_disconnect_in_tcpip_ctx, ctx);
}

static esp_err_t wg_peer_is_up_in_tcpip_ctx(void *arg)
{
    wireguard_ctx_t *ctx = (wireguard_ctx_t *)arg;
    err_t lwip_err;

    lwip_err = wireguardif_peer_is_up(
            ctx->netif,
            wg_local_peer_idx,
            &peer.endpoint_ip,
            &peer.endport_port);

    if (lwip_err != ERR_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t esp_wireguardif_peer_is_up(wireguard_ctx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_netif_tcpip_exec(wg_peer_is_up_in_tcpip_ctx, ctx);
}
