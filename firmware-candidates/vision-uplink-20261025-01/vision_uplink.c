#include "vision_uplink.h"
#include "baby_care_hal.h"
#include <string.h>

#define CHUNK_SLOTS 4U
#define PARTIAL_TIMEOUT_MS 250U
#define VISUAL_CRY_SUSTAIN_MS 3000U

typedef struct { uint8_t data[64]; uint32_t length, received_ms; } Chunk;
static Chunk s_chunks[CHUNK_SLOTS];
static volatile uint8_t s_head, s_tail, s_overflow;
static volatile uint32_t s_generation, s_drops;
static uint32_t s_seen_generation;
static uint8_t s_buffer[64], s_used, s_valid, s_have_sequence;
static uint32_t s_last_byte_ms, s_host, s_sequence, s_expiry_ms, s_received_ms;
static uint32_t s_advertisements[4];
static uint8_t s_ad_count, s_ad_next;
static uint32_t s_label, s_confidence, s_accepted, s_rejected, s_alerts;
static uint8_t s_cry_tracking, s_cry_reported, s_alert_pending;
static uint32_t s_cry_start_ms, s_alert_sequence;
static const uint8_t s_model[32] = {
    0x2a,0xdf,0x7e,0x59,0xdb,0x4a,0x32,0x63,0xb2,0x09,0xfb,0x43,0xf9,0x55,0x19,0x2c,
    0xd2,0xfd,0xda,0xe0,0x85,0x39,0xaf,0x04,0xc1,0x79,0xc9,0xf4,0xe4,0x40,0x85,0xe7
};

static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t crc32(const uint8_t *p, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= p[i];
        for (unsigned bit = 0; bit < 8U; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) != 0U ? 0xedb88320U : 0U);
        }
    }
    return ~crc;
}

static void invalidate(void)
{
    s_valid = 0U;
    s_cry_tracking = s_cry_reported = s_alert_pending = 0U;
}

static void new_session(uint32_t generation)
{
    invalidate();
    s_seen_generation = generation;
    s_host = s_sequence = s_expiry_ms = 0U;
    s_used = s_have_sequence = s_ad_count = s_ad_next = 0U;
}

void VisionUplink_Init(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_head = s_tail = s_overflow = 0U;
    s_generation = s_drops = 0U;
    s_accepted = s_rejected = s_alerts = 0U;
    new_session(0U);
    __set_PRIMASK(primask);
}

void VisionUplink_OnSessionReset(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_generation++;
    s_head = s_tail = s_overflow = 0U;
    __set_PRIMASK(primask);
}

void VisionUplink_OnBytes(const uint8_t *data, uint32_t length)
{
    uint8_t next;
    uint32_t primask;
    if (data == NULL || length == 0U) { return; }
    primask = __get_PRIMASK();
    __disable_irq();
    next = (uint8_t)((s_head + 1U) % CHUNK_SLOTS);
    if (length > sizeof(s_chunks[0].data) || next == s_tail) {
        s_drops++;
        s_overflow = 1U;
        s_head = s_tail = 0U;
    } else {
        memcpy(s_chunks[s_head].data, data, length);
        s_chunks[s_head].length = length;
        s_chunks[s_head].received_ms = HAL_GetTick();
        s_head = next;
    }
    __set_PRIMASK(primask);
}

static uint8_t accept_frame(uint32_t received_ms, uint32_t generation)
{
    const uint8_t *p = s_buffer;
    const uint32_t host = u32(p + 4), seq = u32(p + 8), expiry = u32(p + 12), age = u32(p + 16);
    const uint32_t confidence = (uint32_t)p[20] | ((uint32_t)p[21] << 8);
    const uint32_t label = p[22], flags = p[23];
    const uint32_t remaining = expiry - HAL_GetTick();
    const uint32_t anchor = expiry - (VISION_UPLINK_TTL_MS - age);
    uint8_t advertised = 0U;
    uint32_t primask;
    for (uint8_t i = 0U; i < s_ad_count; ++i) {
        if (s_advertisements[i] == anchor) { advertised = 1U; }
    }
    if (crc32(p, 60U) != u32(p + 60) || host == 0U ||
        memcmp(p + 24, s_model, sizeof(s_model)) != 0 || u32(p + 56) != 1U ||
        label > 4U || confidence > 1000U ||
        (label == 0U ? flags != 0U || confidence != 0U : flags != 1U || confidence < 600U) ||
        age >= VISION_UPLINK_TTL_MS || !advertised || remaining == 0U ||
        remaining > VISION_UPLINK_TTL_MS - age ||
        (s_host != 0U && host != s_host) ||
        (s_have_sequence && (seq - s_sequence == 0U || seq - s_sequence >= 0x80000000U))) {
        s_rejected++;
        invalidate();
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (generation != s_generation) {
        __set_PRIMASK(primask);
        invalidate();
        return 0U;
    }
    /* A valid frame can report observations only; there is no actuator API here. */
    if (!s_valid || (int32_t)(received_ms - s_expiry_ms) >= 0 || label != 1U) {
        s_cry_tracking = s_cry_reported = s_alert_pending = 0U;
    }
    s_host = host;
    s_sequence = seq;
    s_have_sequence = 1U;
    s_label = label;
    s_confidence = confidence;
    s_received_ms = received_ms;
    s_expiry_ms = expiry;
    s_valid = 1U;
    s_accepted++;
    if (label == 1U) {
        if (!s_cry_tracking) { s_cry_start_ms = received_ms; s_cry_tracking = 1U; }
        if (!s_cry_reported && received_ms - s_cry_start_ms >= VISUAL_CRY_SUSTAIN_MS) {
            s_cry_reported = s_alert_pending = 1U;
            s_alert_sequence = seq;
            s_alerts++;
        }
    }
    __set_PRIMASK(primask);
    return 1U;
}

void VisionUplink_Task(void)
{
    Chunk local[CHUNK_SLOTS - 1U];
    uint32_t generation, primask;
    uint8_t count = 0U, overflow;
    primask = __get_PRIMASK();
    __disable_irq();
    generation = s_generation;
    overflow = s_overflow;
    s_overflow = 0U;
    while (s_tail != s_head && count < CHUNK_SLOTS - 1U) {
        local[count++] = s_chunks[s_tail];
        s_tail = (uint8_t)((s_tail + 1U) % CHUNK_SLOTS);
    }
    __set_PRIMASK(primask);
    if (generation != s_seen_generation) { new_session(generation); }
    if (overflow) { s_used = 0U; invalidate(); return; }
    for (uint8_t n = 0U; n < count; ++n) {
        if (s_used && local[n].received_ms - s_last_byte_ms > PARTIAL_TIMEOUT_MS) {
            s_used = 0U;
            invalidate();
        }
        for (uint32_t i = 0U; i < local[n].length; ++i) {
            s_buffer[s_used++] = local[n].data[i];
            s_last_byte_ms = local[n].received_ms;
            while (s_used >= 4U && memcmp(s_buffer, "BV01", 4U) != 0) {
                memmove(s_buffer, s_buffer + 1, --s_used);
            }
            if (s_used == sizeof(s_buffer)) {
                if (accept_frame(local[n].received_ms, generation)) { s_used = 0U; }
                else { memmove(s_buffer, s_buffer + 1, --s_used); }
            }
        }
    }
    if (s_generation != s_seen_generation ||
        (s_valid && (int32_t)(HAL_GetTick() - s_expiry_ms) >= 0)) {
        invalidate();
    }
    if (s_used && HAL_GetTick() - s_last_byte_ms > PARTIAL_TIMEOUT_MS) {
        s_used = 0U;
        invalidate();
    }
}

void VisionUplink_GetSnapshot(VisionUplink_Snapshot *snapshot)
{
    uint32_t primask, remaining;
    if (snapshot == NULL) { return; }
    primask = __get_PRIMASK();
    __disable_irq();
    remaining = s_expiry_ms - HAL_GetTick();
    snapshot->fresh = s_valid && s_seen_generation == s_generation &&
        remaining > 0U && remaining <= VISION_UPLINK_TTL_MS;
    snapshot->host = s_seen_generation == s_generation ? s_host : 0U;
    snapshot->sequence = s_sequence;
    snapshot->label = snapshot->fresh ? s_label : 0U;
    snapshot->confidence = snapshot->fresh ? s_confidence : 0U;
    snapshot->age_ms = snapshot->fresh ? VISION_UPLINK_TTL_MS - remaining : UINT32_MAX;
    snapshot->accepted = s_accepted;
    snapshot->rejected = s_rejected;
    snapshot->drops = s_drops;
    snapshot->alerts = s_alerts;
    __set_PRIMASK(primask);
}

uint32_t VisionUplink_AdvertiseTick(void)
{
    const uint32_t now = HAL_GetTick();
    s_advertisements[s_ad_next] = now;
    s_ad_next = (uint8_t)((s_ad_next + 1U) % 4U);
    if (s_ad_count < 4U) { s_ad_count++; }
    return now;
}

uint8_t VisionUplink_TakeCryAlert(uint32_t *sequence)
{
    VisionUplink_Snapshot snapshot;
    uint32_t primask;
    if (sequence == NULL) { return 0U; }
    primask = __get_PRIMASK();
    __disable_irq();
    VisionUplink_GetSnapshot(&snapshot);
    if (!snapshot.fresh || !s_alert_pending) { __set_PRIMASK(primask); return 0U; }
    *sequence = s_alert_sequence;
    s_alert_pending = 0U;
    __set_PRIMASK(primask);
    return 1U;
}
