#include "ch554t.h"

#include <string.h>
#include <unistd.h>

#include <libusb-1.0/libusb.h>

#define CH554T_VID 0x1fc8
#define CH554T_PID 0x310b

#define CH554T_EP_CMD_OUT 0x02
#define CH554T_EP_DATA_OUT 0x01
#define CH554T_EP_IN 0x82

#define CH554T_IFACE 0
#define CH554T_FRAME_SIZE 64
#define CH554T_CHUNK_SIZE 256
#define CH554T_TIMEOUT_MS 5000
#define CH554T_READBACK_DELAY_US 50000
#define CH554T_ERASE_POLL_DELAY_US 50000
#define CH554T_ERASE_MAX_POLLS 4096

#define CH554T_CMD_SETUP_A 0x09
#define CH554T_CMD_SETUP_B 0x07
#define CH554T_CMD_START_RW 0x05
#define CH554T_CMD_DETECT 0x0B
#define CH554T_CMD_POLL 0x0A

#define CH554T_CMD_ERASE_GRP 0x01
#define CH554T_CMD_ERASE_SUB 0x02
#define CH554T_CMD_FINALIZE_SUB 0x08

struct ch554t_ctx {
    libusb_context *ctx;
    libusb_device_handle *handle;
    int initialized;
    int erase_in_progress;
    int erase_poll_frame_valid;
    uint8_t erase_poll_frame[CH554T_FRAME_SIZE];
    int read_stream_active;
    uint32_t read_stream_next;
    int write_stream_active;
    uint32_t write_stream_next;
};

static struct ch554t_ctx g_ch554t = {0};

static void ch554t_reset_read_stream(void)
{
    g_ch554t.read_stream_active = 0;
    g_ch554t.read_stream_next = 0;
}

static void ch554t_reset_write_stream(void)
{
    g_ch554t.write_stream_active = 0;
    g_ch554t.write_stream_next = 0;
}

static void ch554t_reset_streams(void)
{
    ch554t_reset_read_stream();
    ch554t_reset_write_stream();
}

static void ch554t_reset_erase_state(void)
{
    g_ch554t.erase_in_progress = 0;
    g_ch554t.erase_poll_frame_valid = 0;
    memset(g_ch554t.erase_poll_frame, 0, sizeof(g_ch554t.erase_poll_frame));
}

static uint16_t bswap16_u(uint16_t x)
{
    return (uint16_t)((x >> 8) | (x << 8));
}

static uint32_t bswap32_u(uint32_t x)
{
    return ((x & 0x000000FFu) << 24) |
           ((x & 0x0000FF00u) << 8) |
           ((x & 0x00FF0000u) >> 8) |
           ((x & 0xFF000000u) >> 24);
}

static int ch554t_bulk_cmd(uint8_t *buf, int readback)
{
    int ret;
    int transferred = 0;

    ret = libusb_bulk_transfer(g_ch554t.handle, CH554T_EP_CMD_OUT, buf, CH554T_FRAME_SIZE, &transferred, CH554T_TIMEOUT_MS);
    if (ret != 0 || transferred != CH554T_FRAME_SIZE) return -1;

    if (!readback) return 0;

    usleep(CH554T_READBACK_DELAY_US);
    transferred = 0;
    ret = libusb_bulk_transfer(g_ch554t.handle, CH554T_EP_IN, buf, CH554T_FRAME_SIZE, &transferred, CH554T_TIMEOUT_MS);
    if (ret != 0 || transferred != CH554T_FRAME_SIZE) return -1;
    return 0;
}

static int ch554t_bulk_read(uint8_t *buf, int len)
{
    int ret;
    int transferred = 0;

    ret = libusb_bulk_transfer(g_ch554t.handle, CH554T_EP_IN, buf, len, &transferred, CH554T_TIMEOUT_MS);
    if (ret != 0 || transferred != len) return -1;
    return 0;
}

static int ch554t_bulk_write(const uint8_t *buf, int len)
{
    int ret;
    int transferred = 0;

    ret = libusb_bulk_transfer(g_ch554t.handle, CH554T_EP_DATA_OUT, (unsigned char *)buf, len, &transferred, CH554T_TIMEOUT_MS);
    if (ret != 0 || transferred != len) return -1;
    return 0;
}

static void ch554t_build_setup(uint8_t *buf, uint8_t cmd1)
{
    uint16_t f1168 = bswap16_u(0x0100);
    uint16_t f1172 = bswap16_u(0x03E8);
    uint32_t f1164 = bswap32_u(0x00080000);
    uint32_t f1160 = bswap32_u(0x00EF6013);

    memset(buf, 0, CH554T_FRAME_SIZE);
    buf[0] = 0x00;
    buf[1] = cmd1;
    buf[2] = 0x00;
    buf[3] = 0x00;
    memcpy(&buf[4], &f1168, sizeof(f1168));
    memcpy(&buf[6], &f1172, sizeof(f1172));
    memcpy(&buf[8], &f1164, sizeof(f1164));
    memcpy(&buf[12], &f1160, sizeof(f1160));
    buf[16] = 0x00;
    buf[28] = 0x00;
}

static void ch554t_build_start_rw(uint8_t *buf, uint32_t arg32)
{
    memset(buf, 0, CH554T_FRAME_SIZE);
    buf[0] = 0x00;
    buf[1] = CH554T_CMD_START_RW;
    memcpy(&buf[8], &arg32, sizeof(arg32));
}

static int ch554t_begin_rw_stream(uint32_t addr, int readback, int *stream_active, uint32_t *stream_next)
{
    uint8_t cmd[CH554T_FRAME_SIZE];

    if (*stream_active && addr == *stream_next) return 0;

    ch554t_build_start_rw(cmd, addr);
    if (ch554t_bulk_cmd(cmd, readback) != 0) return -1;

    *stream_active = 1;
    *stream_next = addr;
    return 0;
}

static void ch554t_build_detect(uint8_t *buf)
{
    memset(buf, 0, CH554T_FRAME_SIZE);
    buf[0] = 0x00;
    buf[1] = CH554T_CMD_DETECT;
    buf[2] = 0x03;
    buf[3] = 0x04;
    buf[4] = 0x9F;
    buf[5] = 0xFF;
    buf[6] = 0xFF;
    buf[7] = 0xFF;
}

static void ch554t_build_poll(uint8_t *buf)
{
    memset(buf, 0, CH554T_FRAME_SIZE);
    buf[0] = 0x00;
    buf[1] = CH554T_CMD_POLL;
}

static void ch554t_build_chip_erase(uint8_t *buf)
{
    memset(buf, 0, CH554T_FRAME_SIZE);
    buf[0] = CH554T_CMD_ERASE_GRP;
    buf[1] = CH554T_CMD_ERASE_SUB;
    buf[26] = 0x80;
    buf[27] = 0x00;
}

static void ch554t_build_finalize(uint8_t *buf)
{
    memset(buf, 0, CH554T_FRAME_SIZE);
    buf[0] = CH554T_CMD_ERASE_GRP;
    buf[1] = CH554T_CMD_FINALIZE_SUB;
}

bool ch554t_spi_init(uint8_t ch_type, uint8_t i2cBusSpeed)
{
    int ret;
    int config = -1;
    uint8_t cmd[CH554T_FRAME_SIZE];

    (void)ch_type;
    (void)i2cBusSpeed;

    if (g_ch554t.initialized) return false;

    ret = libusb_init(&g_ch554t.ctx);
    if (ret != 0) return true;

    g_ch554t.handle = libusb_open_device_with_vid_pid(g_ch554t.ctx, CH554T_VID, CH554T_PID);
    if (!g_ch554t.handle) goto fail;

    ret = libusb_get_configuration(g_ch554t.handle, &config);
    if (ret == 0 && config != 1) {
        ret = libusb_set_configuration(g_ch554t.handle, 1);
        if (ret != 0) goto fail;
    }

    ret = libusb_claim_interface(g_ch554t.handle, CH554T_IFACE);
    if (ret != 0) goto fail;

    ch554t_build_setup(cmd, CH554T_CMD_SETUP_A);
    if (ch554t_bulk_cmd(cmd, 1) != 0) goto fail;

    ch554t_build_setup(cmd, CH554T_CMD_SETUP_B);
    if (ch554t_bulk_cmd(cmd, 1) != 0) goto fail;

    g_ch554t.initialized = 1;
    return false;

fail:
    ch554t_spi_shutdown();
    return true;
}

void ch554t_spi_shutdown(void)
{
    uint8_t fin[CH554T_FRAME_SIZE];

    if (g_ch554t.handle) {
        ch554t_build_finalize(fin);
        (void)ch554t_bulk_cmd(fin, 0);
        libusb_release_interface(g_ch554t.handle, CH554T_IFACE);
        libusb_close(g_ch554t.handle);
        g_ch554t.handle = NULL;
    }

    if (g_ch554t.ctx) {
        libusb_exit(g_ch554t.ctx);
        g_ch554t.ctx = NULL;
    }

    g_ch554t.initialized = 0;
    ch554t_reset_erase_state();
    ch554t_reset_streams();
}

int ch554t_get_descriptor(uint8_t *buf)
{
    if (!g_ch554t.handle) return -1;
    return libusb_get_descriptor(g_ch554t.handle, LIBUSB_DT_DEVICE, 0x00, buf, 0x12);
}

int ch554tGetDescriptor(uint8_t *buf)
{
    return ch554t_get_descriptor(buf);
}

int ch554t_read_devid(uint8_t *rxbuf, int n_rx)
{
    uint8_t cmd[CH554T_FRAME_SIZE];
    int i;

    if (!g_ch554t.initialized || n_rx <= 0) return -1;
    ch554t_reset_streams();

    ch554t_build_detect(cmd);
    if (ch554t_bulk_cmd(cmd, 1) != 0) return -1;

    for (i = 0; i < n_rx; i++) rxbuf[i] = 0xFF;
    if (n_rx > 0) rxbuf[0] = cmd[1];
    if (n_rx > 1) rxbuf[1] = cmd[2];
    if (n_rx > 2) rxbuf[2] = cmd[3];

    return 0;
}

int ch554t_read_data(uint32_t from, uint8_t *buf, uint32_t len)
{
    uint32_t remain = len;
    uint32_t offset = 0;
    int chunk;

    if (!g_ch554t.initialized || !buf) return -1;

    ch554t_reset_write_stream();
    if (ch554t_begin_rw_stream(from, 1, &g_ch554t.read_stream_active, &g_ch554t.read_stream_next) != 0) return -1;

    while (remain > 0) {
        chunk = (remain > CH554T_CHUNK_SIZE) ? CH554T_CHUNK_SIZE : (int)remain;
        if (ch554t_bulk_read(&buf[offset], chunk) != 0) return -1;
        offset += (uint32_t)chunk;
        remain -= (uint32_t)chunk;
        g_ch554t.read_stream_next += (uint32_t)chunk;
    }

    return 0;
}

int ch554t_write_data(uint32_t to, const uint8_t *buf, uint32_t len)
{
    uint32_t remain = len;
    uint32_t offset = 0;
    int chunk;

    if (!g_ch554t.initialized || !buf) return -1;
    ch554t_reset_read_stream();

    /*
     * Keep one continuous write stream across sequential calls.
     * IMSProg writes block-by-block; restarting 00 05 on every block/chunk
     * can make some CH554T firmware loop back to an earlier address.
     */
    if (ch554t_begin_rw_stream(to, 0, &g_ch554t.write_stream_active, &g_ch554t.write_stream_next) != 0) return -1;

    while (remain > 0) {
        chunk = (remain > CH554T_CHUNK_SIZE) ? CH554T_CHUNK_SIZE : (int)remain;
        if (ch554t_bulk_write(&buf[offset], chunk) != 0) return -1;
        offset += (uint32_t)chunk;
        remain -= (uint32_t)chunk;
        g_ch554t.write_stream_next += (uint32_t)chunk;
    }

    return 0;
}

int ch554t_chip_erase_start(void)
{
    uint8_t cmd[CH554T_FRAME_SIZE];

    if (!g_ch554t.initialized) return -1;
    ch554t_reset_streams();
    ch554t_reset_erase_state();

    ch554t_build_chip_erase(cmd);
    if (ch554t_bulk_cmd(cmd, 0) != 0) {
        ch554t_reset_erase_state();
        return -1;
    }

    /*
     * Original software sends first poll as 00 0A with the erase frame payload.
     * Then it reuses each poll response as the next poll request (opcode bytes updated).
     */
    memcpy(g_ch554t.erase_poll_frame, cmd, CH554T_FRAME_SIZE);
    g_ch554t.erase_poll_frame[0] = 0x00;
    g_ch554t.erase_poll_frame[1] = CH554T_CMD_POLL;
    g_ch554t.erase_poll_frame_valid = 1;
    g_ch554t.erase_in_progress = 1;
    return 0;
}

int ch554t_chip_erase_poll(void)
{
    uint8_t resp[CH554T_FRAME_SIZE];

    if (!g_ch554t.initialized) return -1;
    if (!g_ch554t.erase_in_progress) return 0;

    if (g_ch554t.erase_poll_frame_valid) memcpy(resp, g_ch554t.erase_poll_frame, CH554T_FRAME_SIZE);
    else ch554t_build_poll(resp);
    resp[0] = 0x00;
    resp[1] = CH554T_CMD_POLL;
    usleep(CH554T_ERASE_POLL_DELAY_US);
    if (ch554t_bulk_cmd(resp, 1) != 0) {
        ch554t_reset_erase_state();
        return -1;
    }
    if ((resp[0] & 1) != 0) {
        memcpy(g_ch554t.erase_poll_frame, resp, CH554T_FRAME_SIZE);
        g_ch554t.erase_poll_frame[0] = 0x00;
        g_ch554t.erase_poll_frame[1] = CH554T_CMD_POLL;
        g_ch554t.erase_poll_frame_valid = 1;
        return 1;
    }

    ch554t_reset_erase_state();
    return 0;
}

int ch554t_chip_erase_finalize(void)
{
    if (!g_ch554t.initialized) return -1;
    ch554t_reset_erase_state();
    ch554t_reset_streams();
    return 0;
}

int ch554t_chip_erase(void)
{
    int i;
    int ret;

    ret = ch554t_chip_erase_start();
    if (ret != 0) return -1;

    for (i = 0; i < CH554T_ERASE_MAX_POLLS; i++) {
        ret = ch554t_chip_erase_poll();
        if (ret < 0) return -1;
        if (ret == 0) return 0;
    }

    ch554t_chip_erase_finalize();
    return -1;
}
