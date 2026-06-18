#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "wmm.h"

#define PI_CONST 3.14159265359f
#define RADIANS_TO_DEGREES 0.017453292f
#define DEGREES_TO_RADIANS (PI_CONST / 180.0f)
#define A_CONST 6378.137f
#define A2_CONST (A_CONST * A_CONST)
#define B_CONST 6356.7523142f
#define B2_CONST (B_CONST * B_CONST)
#define RE_CONST 6371.2f
#define A4_CONST (A2_CONST * A2_CONST)
#define B4_CONST (B2_CONST * B2_CONST)
#define C2_CONST (A2_CONST - B2_CONST)
#define C4_CONST (A4_CONST - B4_CONST)
#define COEFFICIENTS_COUNT 90U

static float c[13][13];
static float cd[13][13];
static float k[13][13];
static float snorm[169];
static float fn[13];
static float fm[13];
extern const uint8_t wmm_cof_entries_encoded[];
static wmm_cof_record_t wmm_cof_entries[COEFFICIENTS_COUNT];

static float convert_varint_to_float(char **bytes);

float wmm_get_date(uint8_t year, uint8_t month, uint8_t date)
{
    float days_in_year = 365.0f;
    if (year % 4U == 0U) days_in_year = 366.0f;
    return (float)year + 2000.0f + (float)(month - 1U) / 12.0f + (float)(date - 1U) / days_in_year;
}

static float convert_varint_to_float(char **bytes)
{
    float result;
    int32_t result_int;
    bool negative = false;
    bool first_byte = true;
    uint8_t shift;
    do {
        if (first_byte) {
            if (**bytes & 0x40) negative = true;
            result_int = **bytes & 0x3f;
            shift = 6U;
            first_byte = false;
        } else {
            result_int += (uint32_t)(**bytes & 0x7f) << shift;
            shift += 7U;
        }
        if ((**bytes & 0x80) == 0U) {
            (*bytes)++;
            break;
        }
        (*bytes)++;
    } while (true);
    result = ((float)result_int) / 10.0f;
    return negative ? -result : result;
}

void wmm_init(void)
{
    uint8_t j;
    uint8_t m;
    uint8_t n;
    uint8_t D2wmm;
    float gnm;
    float hnm;
    float dgnm;
    float dhnm;
    float flnmj;
    uint8_t i;
    char *bytes = (char *)&wmm_cof_entries_encoded[0];

    for (i = 0U; i < COEFFICIENTS_COUNT; i++) {
        wmm_cof_entries[i].gnm = convert_varint_to_float(&bytes);
        wmm_cof_entries[i].hnm = convert_varint_to_float(&bytes);
        wmm_cof_entries[i].dgnm = convert_varint_to_float(&bytes);
        wmm_cof_entries[i].dhnm = convert_varint_to_float(&bytes);
    }

    c[0][0] = 0.0f;
    cd[0][0] = 0.0f;
    j = 0U;
    for (n = 1U; n <= 12U; n++) {
        for (m = 0U; m <= n; m++) {
            gnm = wmm_cof_entries[j].gnm;
            hnm = wmm_cof_entries[j].hnm;
            dgnm = wmm_cof_entries[j].dgnm;
            dhnm = wmm_cof_entries[j].dhnm;
            j++;
            c[m][n] = gnm;
            cd[m][n] = dgnm;
            if (m != 0U) {
                c[n][m - 1U] = hnm;
                cd[n][m - 1U] = dhnm;
            }
        }
    }

    *snorm = 1.0f;
    for (n = 1U; n <= 12U; n++) {
        *(snorm + n) = *(snorm + n - 1U) * (float)(2U * n - 1U) / (float)n;
        j = 2U;
        m = 0U;
        for (D2wmm = n - m + 1U; D2wmm > 0U; D2wmm--) {
            k[m][n] = (float)(((n - 1U) * (n - 1U)) - (m * m)) / (float)((2U * n - 1U) * (2U * n - 3U));
            if (m > 0U) {
                flnmj = (float)((n - m + 1U) * j) / (float)(n + m);
                *(snorm + n + m * 13U) = *(snorm + n + (m - 1U) * 13U) * sqrt(flnmj);
                j = 1U;
                c[n][m - 1U] = *(snorm + n + m * 13U) * c[n][m - 1U];
                cd[n][m - 1U] = *(snorm + n + m * 13U) * cd[n][m - 1U];
            }
            c[m][n] = *(snorm + n + m * 13U) * c[m][n];
            cd[m][n] = *(snorm + n + m * 13U) * cd[m][n];
            m += 1U;
        }
        fn[n] = (float)(n + 1U);
        fm[n] = (float)n;
    }
    k[1][1] = 0.0f;
}
