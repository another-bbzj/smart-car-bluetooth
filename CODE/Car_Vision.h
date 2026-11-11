#ifndef CAR_VISION_H
#define CAR_VISION_H
#include <stdint.h>

/* Orange Pi -> F407, independent of Bluetooth, all wire integers big-endian.
 * A5 5A | version:u8 type:u8 length:u16 seq:u16 session:u32 age_ms:u16
 * | payload | CRC16-CCITT-FALSE(version through payload).
 * HELLO=0 (length 0, age 0), TRACK=1 (16), SUDOKU_TILE=2 (10).
 * HELLO may bind a new nonzero session only while the chassis is stopped.
 * No message in this interface can start motors or write PWM. */
#define CAR_VISION_MAX_PAYLOAD 32U
#define CAR_VISION_MAX_FRAME   (16U+CAR_VISION_MAX_PAYLOAD)
#define CAR_VISION_FRAME_GAP_MS 50U
#define CAR_VISION_MAX_AGE_MS   200U
enum { VISION_LATERAL_VALID=1, VISION_HEADING_VALID=2,
       VISION_CURVATURE_VALID=4, VISION_DISTANCE_VALID=8 };
enum { VISION_ELEMENT_NONE, VISION_ELEMENT_RING, VISION_ELEMENT_CROSS,
       VISION_ELEMENT_GAP, VISION_ELEMENT_GREEN, VISION_ELEMENT_YELLOW,
       VISION_ELEMENT_RED, VISION_ELEMENT_FINISH, VISION_ELEMENT_TAG };
typedef struct {
    uint32_t received_ms, session;
    uint16_t sequence, processing_age_ms, valid_mask;
    float lateral_m, heading_rad, curvature_inv_m, distance_m, confidence;
    uint8_t element, tag_id;
} CarVisionObservation;
typedef struct {
    uint32_t received_ms, session;
    uint16_t sequence;
    uint8_t cells[9], valid;
} CarVisionTile;
typedef struct {
    uint8_t buffer[CAR_VISION_MAX_FRAME], used, session_valid, have_sequence, have_observation;
    uint16_t last_sequence;
    uint32_t last_byte_ms, session, accepted_frames, bad_frames, rejected_frames;
    CarVisionObservation observation;
    CarVisionTile tiles[4];
} CarVision;
void CarVision_Init(CarVision *v);
void CarVision_Invalidate(CarVision *v);
uint16_t CarVision_Crc16(uint16_t crc, uint8_t byte);
/* All mutations must run in one owner context; readers need a snapshot boundary. */
void CarVision_Feed(CarVision *v, uint8_t byte, uint32_t now_ms, uint8_t allow_hello);
/* Fresh transport data can still have valid_mask=0 (no road detected). */
uint8_t CarVision_GetObservation(const CarVision *v, uint32_t now_ms, CarVisionObservation *out);
/* Tiles persist for this session; caller decides when the four-tile puzzle is complete. */
uint8_t CarVision_GetTile(const CarVision *v, uint8_t index, uint32_t now_ms, CarVisionTile *out);
#endif
