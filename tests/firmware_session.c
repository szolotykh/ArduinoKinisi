// File: tests/firmware_session.c
// Optional host fixture: real firmware framing/INIT/clock logic, stubbed hardware commands.
#include "connection.h"
#include "initialization.h"
#include <assert.h>
#include <string.h>

static connection_t connection;
static uint64_t now_us;
static uint8_t replies[8][256], lengths[8], head, tail;

/** Supply deterministic controller uptime to the production session. */
static uint64_t now(void) { return now_us; }
/** Capture one complete firmware frame without changing its wire representation. */
static bool capture(uint8_t* bytes, uint8_t length) {
    assert(tail < 8);
    memcpy(replies[tail], bytes, length);
    lengths[tail++] = length;
    return true;
}
/** Stub physical resources only; INIT validation and response come from real firmware. */
static uint8_t handle(controller_command_t* cmd, protocol_send_fn reply) {
    if (cmd->commandType == INIT) {
        uint8_t error = initialization_validate(cmd);
        if (error != RESPONSE_OK) return error;
        init_response identity = initialization_response();
        reply((uint8_t*)&identity, sizeof(identity));
        return RESPONSE_OK;
    }
    if (cmd->commandType == STOP_MOTOR) return RESPONSE_OK;
    if (cmd->commandType == GET_ENCODER_VALUE) {
        uint8_t value[] = {0xef, 0xbe};
        reply(value, sizeof(value));
        return RESPONSE_OK;
    }
    if (cmd->commandType == GET_PLATFORM_ODOMETRY) return RESPONSE_ODOMETRY_NOT_INITIALIZED;
    return RESPONSE_UNKNOWN_COMMAND;
}
/** Start a fresh production firmware session. */
void bridge_reset(void) {
    head = tail = 0;
    now_us = 1000000;
    connection_init(&connection, handle, capture, now);
}
/** Feed a complete master write and advance the firmware command task. */
void bridge_send(const uint8_t* data, size_t length) {
    assert(length == (size_t)data[0] + 1 && connection_can_receive(&connection));
    head = tail = 0;
    connection_receive(&connection, data + 1, length - 1);
    for (unsigned i = 0; i < 8; ++i) { now_us += 1000; connection_poll(&connection); }
}
/** Return one frame, matching the fixed-transaction Wire adapter boundary. */
size_t bridge_read(uint8_t* data) {
    if (head == tail) return 0;
    size_t length = lengths[head];
    memcpy(data, replies[head++], length);
    return length;
}
