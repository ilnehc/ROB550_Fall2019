#ifndef _ENCODE_BYTES
#define _ENCODE_BYTES

#include <stdlib.h>
#include <string.h>

// decode functions are designed to be able to safely decode off the
// end of the buffer (they return 0).
//
// All encode functions support out==NULL, which allows the size of a serialized object
// to be determined.

//////////////////////////////////////////////////////////////////
// N
static inline void encodeN(uint8_t *out, uint32_t *outpos, const uint8_t *in, uint32_t inlen)
{
    if (out)
        memcpy(&out[(*outpos)], in, inlen);
    (*outpos) += inlen;
}


// void decodeN(const uint8_t *in, uint32_t *inpos, uint32_t inlen, uint8_t *out, uint32_t outpos);


//////////////////////////////////////////////////////////////////
// u8
static inline void encode_u8(uint8_t *out, uint32_t *outpos, uint8_t data)
{
    if (out)
        out[(*outpos)++] = data;
    else
        (*outpos) += 1;
}

static inline uint8_t decode_u8(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    if ((*inpos) + 1 > inlen)
        return 0;

    uint8_t v = in[(*inpos)++];
    return v;
}

//////////////////////////////////////////////////////////////////
// u16
static inline void encode_u16(uint8_t *out, uint32_t *outpos, uint16_t data)
{
    if (out) {
        out[(*outpos)++] = (data >> 8) & 0xff;
        out[(*outpos)++] = (data >> 0) & 0xff;
    } else {
        (*outpos) += 2;
    }
}

static inline uint16_t decode_u16(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    if ((*inpos) + 2 > inlen)
        return 0;

    uint16_t v = 0;

    v += in[(*inpos)++] << 8;
    v += in[(*inpos)++] << 0;

    return v;
}

//////////////////////////////////////////////////////////////////
// s16
static inline int16_t decode_s16(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    if ((*inpos) + 2 > inlen)
        return 0;

    uint16_t v = 0;

    v += in[(*inpos)++] << 8;
    v += in[(*inpos)++] << 0;

    return v;
}

//////////////////////////////////////////////////////////////////
// u32
static inline void encode_u32(uint8_t *out, uint32_t *outpos, uint32_t data)
{
    if (out) {
        out[(*outpos)++] = (data >> 24) & 0xff;
        out[(*outpos)++] = (data >> 16) & 0xff;
        out[(*outpos)++] = (data >> 8) & 0xff;
        out[(*outpos)++] = (data >> 0) & 0xff;
    } else {
        (*outpos)+=4;
    }
}

static inline uint32_t decode_u32(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    if ((*inpos) + 4 > inlen)
        return 0;

    uint32_t v = 0;
    v += in[(*inpos)++] << 24;
    v += in[(*inpos)++] << 16;
    v += in[(*inpos)++] << 8;
    v += in[(*inpos)++] << 0;

    return v;
}

//////////////////////////////////////////////////////////////////
// s32
static inline void encode_s32(uint8_t *out, uint32_t *outpos, int32_t data)
{
    if (out) {
        out[(*outpos)++] = (data >> 24) & 0xff;
        out[(*outpos)++] = (data >> 16) & 0xff;
        out[(*outpos)++] = (data >> 8) & 0xff;
        out[(*outpos)++] = (data >> 0) & 0xff;
    } else {
        (*outpos)+=4;
    }
}

static inline int32_t decode_s32(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    if ((*inpos) + 4 > inlen)
        return 0;

    uint32_t v = 0;
    v += in[(*inpos)++] << 24;
    v += in[(*inpos)++] << 16;
    v += in[(*inpos)++] << 8;
    v += in[(*inpos)++] << 0;

    return v;
}

//////////////////////////////////////////////////////////////////
// u64
static inline void encode_u64(uint8_t *out, uint32_t *outpos, uint64_t data)
{
    if (out) {
        out[(*outpos)++] = (data >> 56) & 0xff;
        out[(*outpos)++] = (data >> 48) & 0xff;
        out[(*outpos)++] = (data >> 40) & 0xff;
        out[(*outpos)++] = (data >> 32) & 0xff;
        out[(*outpos)++] = (data >> 24) & 0xff;
        out[(*outpos)++] = (data >> 16) & 0xff;
        out[(*outpos)++] = (data >> 8)  & 0xff;
        out[(*outpos)++] = (data >> 0)  & 0xff;
    } else {
        (*outpos)+=8;
    }
}

static inline uint64_t decode_u64(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    if ((*inpos) + 8 > inlen)
        return 0;

    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
        v = (v << 8) | in[(*inpos)++];
    }
    return v;
}

//////////////////////////////////////////////////////////////////
// s64
static inline void encode_s64(uint8_t *out, uint32_t *outpos, int64_t data)
{
    if (out) {
        out[(*outpos)++] = (data >> 56) & 0xff;
        out[(*outpos)++] = (data >> 48) & 0xff;
        out[(*outpos)++] = (data >> 40) & 0xff;
        out[(*outpos)++] = (data >> 32) & 0xff;
        out[(*outpos)++] = (data >> 24) & 0xff;
        out[(*outpos)++] = (data >> 16) & 0xff;
        out[(*outpos)++] = (data >> 8)  & 0xff;
        out[(*outpos)++] = (data >> 0)  & 0xff;
    } else {
        (*outpos)+=8;
    }
}

static inline uint64_t decode_s64(const uint8_t *in, uint32_t *inpos, int32_t inlen)
{
    if ((*inpos) + 8 > inlen)
        return 0;

    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
        v = (v << 8) | in[(*inpos)++];
    }
    return v;
}

//////////////////////////////////////////////////////////////////
// f32
static inline void encode_f32(uint8_t *out, uint32_t *outpos, float data)
{
    union { uint32_t i;
        float f;
    } u;

    u.f = data;
    encode_u32(out, outpos, u.i);
}


static inline float decode_f32(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    union { uint32_t i;
        float f;
    } u;

    u.i = decode_u32(in, inpos, inlen);
    return u.f;
}

//////////////////////////////////////////////////////////////////
// f64
static inline void encode_f64(uint8_t *out, uint32_t *outpos, float data)
{
    union { uint64_t i;
        double f;
    } u;

    u.f = data;
    encode_u64(out, outpos, u.i);
}


static inline double decode_f64(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    union { uint64_t i;
        double f;
    } u;

    u.i = decode_u64(in, inpos, inlen);
    return u.f;
}

//////////////////////////////////////////////////////////////////
// string_u32
static inline void encode_string_u32(uint8_t *out, uint32_t *outpos, const char *s)
{
    uint32_t len = strlen(s);
    encode_u32(out, outpos, len);
    encodeN(out, outpos, (uint8_t*) s, len);
}

static inline char *decode_string_u32(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    uint32_t len = decode_u32(in, inpos, inlen);

    if ((*inpos) + len > inlen)
        return NULL;

    char *s = malloc(len + 1);
    memcpy(s, &in[*inpos], len);
    s[len] = 0;
    (*inpos) += len;
    return s;
}

#endif
