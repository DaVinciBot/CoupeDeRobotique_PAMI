/*
 * Passerelle Wi-Fi 6E <-> UART des PAMI d'attaque : voir bridge.h et
 * PAMI/lib/com/BRIDGE.md.
 *
 * Bibliothèques Infineon (ModusToolbox) : HAL (cyhal), Wi-Fi Connection
 * Manager (cy_wcm), Secure Sockets (cy_socket), abstraction RTOS (cy_rtos),
 * comme dans l'exemple officiel mtb-example-threadx-wifi-udp-server.
 */
#include "bridge.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "cyhal.h"
#include "cybsp.h"
#include "cy_secure_sockets.h"
#include "cy_wcm.h"
#include "cy_wcm_error.h"

#include "bridge_config.h"
#include "frame.h"

#if BRIDGE_DEBUG
#define LOG(...) printf(__VA_ARGS__)
#else
#define LOG(...) do { } while (0)
#endif

/* Tâches */
#define NET_TASK_STACK_SIZE   (1024 * 5)
#define UART_TASK_STACK_SIZE  (1024 * 3)
static uint64_t net_task_stack[NET_TASK_STACK_SIZE / 8];
static uint64_t uart_task_stack[UART_TASK_STACK_SIZE / 8];
static cy_thread_t net_task_handle;
static cy_thread_t uart_task_handle;

/* UART vers l'ESP32 */
static cyhal_uart_t uart;
static uint8_t uart_rx_buffer[1024];  /* ~11 ms à 921600 bauds */
static cy_mutex_t uart_mutex;         /* une trame à la fois sur l'UART */

/* Réseau */
static cy_socket_t sock;
static bool sock_open = false;
static cy_mutex_t sock_mutex;         /* socket (tâche réseau / tâche UART) */
static cy_socket_sockaddr_t host_addr;
static cy_mutex_t host_mutex;         /* adresse de l'hôte (réception / envoi) */

/* État, envoyé à l'ESP32 dans BRIDGE_STATUS */
static volatile bool wifi_connected = false;
static volatile bool host_known = false;
static volatile int8_t rssi_dbm = 0;

/*******************************************************************************
 * UART
 ******************************************************************************/

static void uart_write(const uint8_t *data, size_t len)
{
    cy_rtos_get_mutex(&uart_mutex, CY_RTOS_NEVER_TIMEOUT);
    size_t n = len;
    cyhal_uart_write(&uart, (void *)data, &n);
    cy_rtos_set_mutex(&uart_mutex);
}

/* BRIDGE_STATUS : | 200 | wifi_connected | host_known | rssi | */
static void send_status(void)
{
    uint8_t msg[4];
    uint8_t frame[sizeof(msg) + FRAME_OVERHEAD];
    msg[0] = (uint8_t)BRIDGE_STATUS_ID;
    msg[1] = wifi_connected ? 1u : 0u;
    msg[2] = host_known ? 1u : 0u;
    msg[3] = (uint8_t)rssi_dbm;
    size_t len = frame_encode(msg, sizeof(msg), frame);
    uart_write(frame, len);
}

static cy_rslt_t uart_init(void)
{
    const cyhal_uart_cfg_t cfg = {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = uart_rx_buffer,
        .rx_buffer_size = sizeof(uart_rx_buffer),
    };
    cy_rslt_t result = cyhal_uart_init(&uart, BRIDGE_UART_TX_PIN, BRIDGE_UART_RX_PIN,
                                       NC, NC, NULL, &cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }
    uint32_t actual = 0;
    result = cyhal_uart_set_baud(&uart, BRIDGE_UART_BAUD, &actual);
    LOG("[UART] ESP32 : %" PRIu32 " bauds demandes, %" PRIu32 " obtenus\n",
        (uint32_t)BRIDGE_UART_BAUD, actual);
    return result;
}

/*******************************************************************************
 * Réseau
 ******************************************************************************/

static void print_ip(const char *label, uint32_t v4)
{
    LOG("%s %u.%u.%u.%u\n", label, (unsigned)(v4 & 0xFFu), (unsigned)((v4 >> 8) & 0xFFu),
        (unsigned)((v4 >> 16) & 0xFFu), (unsigned)((v4 >> 24) & 0xFFu));
}

/* L'expéditeur devient l'hôte (on lui répondra sur BRIDGE_HOST_PORT). */
static void learn_host(const cy_socket_sockaddr_t *peer)
{
    cy_rtos_get_mutex(&host_mutex, CY_RTOS_NEVER_TIMEOUT);
    bool changed = !host_known || (host_addr.ip_address.ip.v4 != peer->ip_address.ip.v4);
    host_addr = *peer;
    host_addr.port = BRIDGE_HOST_PORT;
    host_known = true;
    cy_rtos_set_mutex(&host_mutex);
    if (changed)
    {
        print_ip("[UDP] hote :", peer->ip_address.ip.v4);
    }
}

/* Trame texte de la Jetson ("5|id|x|y|yaw..." commence par un chiffre). */
static bool is_jetson_text(const uint8_t *data, uint32_t len)
{
    return (len > 0u) && (data[0] >= (uint8_t)'0') && (data[0] <= (uint8_t)'9');
}

/* Recopie le texte sur l'UART en morceaux JETSON_TEXT (une trame chacun). */
static void forward_text(const uint8_t *text, uint32_t len)
{
    static uint8_t msg[FRAME_MAX_MESSAGE];
    static uint8_t frame[FRAME_MAX];
    uint8_t index = 0;
    uint32_t done = 0;
    while (done < len)
    {
        uint32_t n = len - done;
        if (n > BRIDGE_TEXT_CHUNK)
        {
            n = BRIDGE_TEXT_CHUNK;
        }
        msg[0] = (uint8_t)BRIDGE_JETSON_TEXT_ID;
        msg[1] = index++;
        msg[2] = (done + n == len) ? 1u : 0u;
        memcpy(&msg[3], &text[done], n);
        size_t flen = frame_encode(msg, (size_t)n + 3u, frame);
        uart_write(frame, flen);
        done += n;
    }
}

/* Wi-Fi -> UART. Un datagramme est :
 *  - soit une trame binaire des PAMI : recopiée telle quelle si elle est
 *    valide et vient de l'hôte (ID < 128) ;
 *  - soit du texte de la Jetson : recopié par morceaux (forward_text).
 * Appelé par la tâche des sockets (pile réduite) : tampon statique. */
static cy_rslt_t udp_recv_handler(cy_socket_t socket_handle, void *arg)
{
    (void)arg;
    static uint8_t buf[BRIDGE_UDP_MAX];
    uint32_t received = 0;
    cy_socket_sockaddr_t peer;
    memset(&peer, 0, sizeof(peer));

    cy_rslt_t result = cy_socket_recvfrom(socket_handle, buf, sizeof(buf),
                                          CY_SOCKET_FLAGS_NONE, &peer, NULL, &received);
    if ((result != CY_RSLT_SUCCESS) || (received == 0u))
    {
        return result;
    }

    if (frame_decode(buf, received) > 0u)
    {
        if (buf[0] >= BRIDGE_FIRST_PAMI_MSG_ID)
        {
            return CY_RSLT_SUCCESS;  /* HELLO ou état d'un autre PAMI : pas pour nous */
        }
        learn_host(&peer);
        uart_write(buf, received);
    }
    else if (is_jetson_text(buf, received))
    {
        learn_host(&peer);
        forward_text(buf, received);
    }
    /* sinon : trame abîmée ou inconnue, ignorée */
    return CY_RSLT_SUCCESS;
}

static void close_socket(void)
{
    cy_rtos_get_mutex(&sock_mutex, CY_RTOS_NEVER_TIMEOUT);
    if (sock_open)
    {
        cy_socket_delete(sock);
        sock_open = false;
    }
    cy_rtos_set_mutex(&sock_mutex);
}

static cy_rslt_t open_socket(const cy_wcm_ip_address_t *ip)
{
    cy_socket_t s;
    cy_rslt_t result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_DGRAM,
                                        CY_SOCKET_IPPROTO_UDP, &s);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    cy_socket_opt_callback_t recv_option = {
        .callback = udp_recv_handler,
        .arg = NULL,
    };
    result = cy_socket_setsockopt(s, CY_SOCKET_SOL_SOCKET, CY_SOCKET_SO_RECEIVE_CALLBACK,
                                  &recv_option, sizeof(recv_option));
    if (result == CY_RSLT_SUCCESS)
    {
        cy_socket_sockaddr_t local;
        memset(&local, 0, sizeof(local));
        local.ip_address.ip.v4 = ip->ip.v4;
        local.ip_address.version = CY_SOCKET_IP_VER_V4;
        local.port = BRIDGE_PAMI_PORT;
        result = cy_socket_bind(s, &local, sizeof(local));
    }
    if (result != CY_RSLT_SUCCESS)
    {
        cy_socket_delete(s);
        return result;
    }

    cy_rtos_get_mutex(&sock_mutex, CY_RTOS_NEVER_TIMEOUT);
    sock = s;
    sock_open = true;
    cy_rtos_set_mutex(&sock_mutex);
    LOG("[UDP] ecoute sur le port %u\n", (unsigned)BRIDGE_PAMI_PORT);
    return CY_RSLT_SUCCESS;
}

static cy_rslt_t join_wifi(cy_wcm_ip_address_t *ip)
{
    cy_wcm_connect_params_t params;
    memset(&params, 0, sizeof(params));
    strncpy((char *)params.ap_credentials.SSID, BRIDGE_WIFI_SSID,
            sizeof(params.ap_credentials.SSID) - 1u);
    strncpy((char *)params.ap_credentials.password, BRIDGE_WIFI_PASSWORD,
            sizeof(params.ap_credentials.password) - 1u);
    params.ap_credentials.security = BRIDGE_WIFI_SECURITY;
    return cy_wcm_connect_ap(&params, ip);
}

static void net_task(cy_thread_arg_t arg)
{
    (void)arg;
    cy_wcm_config_t wcm_config = { .interface = CY_WCM_INTERFACE_TYPE_STA };
    if ((cy_wcm_init(&wcm_config) != CY_RSLT_SUCCESS) || (cy_socket_init() != CY_RSLT_SUCCESS))
    {
        LOG("[WiFi] initialisation impossible\n");
        CY_ASSERT(0);
    }
    if (strlen(BRIDGE_WIFI_SSID) == 0u)
    {
        LOG("[WiFi] pas de reseau : copier bridge_secrets.example.h en bridge_secrets.h\n");
        for (;;)
        {
            cy_rtos_delay_milliseconds(1000);
        }
    }

    for (;;)
    {
        if (!cy_wcm_is_connected_to_ap())
        {
            if (wifi_connected)
            {
                LOG("[WiFi] reseau perdu, reconnexion...\n");
                wifi_connected = false;
                rssi_dbm = 0;
            }
            close_socket();  /* l'adresse IP peut changer : socket refait */

            cy_wcm_ip_address_t ip;
            memset(&ip, 0, sizeof(ip));
            cy_rslt_t result = join_wifi(&ip);
            if (result == CY_RSLT_SUCCESS)
            {
                LOG("[WiFi] connecte a '%s'\n", BRIDGE_WIFI_SSID);
                print_ip("[WiFi] adresse :", ip.ip.v4);
                result = open_socket(&ip);
                if (result == CY_RSLT_SUCCESS)
                {
                    wifi_connected = true;
                    continue;
                }
                LOG("[UDP] socket impossible (0x%08" PRIx32 ")\n", (uint32_t)result);
                cy_wcm_disconnect_ap();
            }
            else
            {
                LOG("[WiFi] connexion echouee (0x%08" PRIx32 ")\n", (uint32_t)result);
            }
            cy_rtos_delay_milliseconds(BRIDGE_WIFI_RETRY_MS);
            continue;
        }

        /* Connecté : puissance du signal, pour BRIDGE_STATUS */
        cy_wcm_associated_ap_info_t info;
        if (cy_wcm_get_associated_ap_info(&info) == CY_RSLT_SUCCESS)
        {
            int16_t rssi = info.signal_strength;
            rssi_dbm = (int8_t)((rssi < -127) ? -127 : ((rssi > 0) ? 0 : rssi));
        }
        cy_rtos_delay_milliseconds(BRIDGE_STATUS_PERIOD_MS);
    }
}

/*******************************************************************************
 * UART -> Wi-Fi
 ******************************************************************************/

/* Envoie le message en UDP : à l'hôte s'il est connu, sinon en diffusion. */
static void forward_to_host(const uint8_t *msg, size_t size)
{
    if (!wifi_connected)
    {
        return;  /* pas de réseau : trame perdue (l'ESP32 renvoie 10 fois/s) */
    }
    uint8_t frame[FRAME_MAX];
    size_t len = frame_encode(msg, size, frame);
    if (len == 0u)
    {
        return;
    }

    cy_socket_sockaddr_t dest;
    cy_rtos_get_mutex(&host_mutex, CY_RTOS_NEVER_TIMEOUT);
    if (host_known)
    {
        dest = host_addr;
    }
    else
    {
        memset(&dest, 0, sizeof(dest));
        dest.ip_address.version = CY_SOCKET_IP_VER_V4;
        dest.ip_address.ip.v4 = 0xFFFFFFFFu;  /* 255.255.255.255 */
        dest.port = BRIDGE_HOST_PORT;
    }
    cy_rtos_set_mutex(&host_mutex);

    cy_rtos_get_mutex(&sock_mutex, CY_RTOS_NEVER_TIMEOUT);
    if (sock_open)
    {
        uint32_t sent = 0;
        cy_socket_sendto(sock, frame, (uint32_t)len, CY_SOCKET_FLAGS_NONE, &dest,
                         sizeof(dest), &sent);
    }
    cy_rtos_set_mutex(&sock_mutex);
}

static void uart_task(cy_thread_arg_t arg)
{
    (void)arg;
    static frame_parser_t parser;
    frame_parser_init(&parser);
    cy_time_t last_status = 0;

    for (;;)
    {
        uint8_t chunk[64];
        size_t n = cyhal_uart_readable(&uart);
        if (n > 0u)
        {
            if (n > sizeof(chunk))
            {
                n = sizeof(chunk);
            }
            if (cyhal_uart_read(&uart, chunk, &n) == CY_RSLT_SUCCESS)
            {
                for (size_t i = 0; i < n; ++i)
                {
                    size_t size = frame_parser_feed(&parser, chunk[i]);
                    if (size > 0u)
                    {
                        forward_to_host(parser.msg, size);
                    }
                }
            }
        }
        else
        {
            cy_rtos_delay_milliseconds(1);
        }

        cy_time_t now = 0;
        cy_rtos_get_time(&now);
        if (now - last_status >= BRIDGE_STATUS_PERIOD_MS)
        {
            last_status = now;
            send_status();  /* même sans réseau : l'ESP32 sait que le module répond */
        }
    }
}

/*******************************************************************************
 * Démarrage
 ******************************************************************************/

cy_rslt_t bridge_start(void)
{
    cy_rslt_t result = cy_rtos_init_mutex(&uart_mutex);
    if (result == CY_RSLT_SUCCESS)
    {
        result = cy_rtos_init_mutex(&sock_mutex);
    }
    if (result == CY_RSLT_SUCCESS)
    {
        result = cy_rtos_init_mutex(&host_mutex);
    }
    if (result == CY_RSLT_SUCCESS)
    {
        result = uart_init();
    }
    if (result != CY_RSLT_SUCCESS)
    {
        LOG("[Bridge] initialisation impossible (0x%08" PRIx32 ")\n", (uint32_t)result);
        return result;
    }

    result = cy_rtos_thread_create(&uart_task_handle, uart_task, "bridge_uart",
                                   &uart_task_stack, UART_TASK_STACK_SIZE,
                                   CY_RTOS_PRIORITY_ABOVENORMAL, 0);
    if (result == CY_RSLT_SUCCESS)
    {
        result = cy_rtos_thread_create(&net_task_handle, net_task, "bridge_net",
                                       &net_task_stack, NET_TASK_STACK_SIZE,
                                       CY_RTOS_PRIORITY_NORMAL, 0);
    }
    return result;
}
