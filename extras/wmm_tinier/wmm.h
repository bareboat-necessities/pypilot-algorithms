#ifndef WMM_H
#define WMM_H

#include <stdint.h>

#define WMM_EPOCH 2025.0f

typedef struct
{
    float gnm;
    float hnm;
    float dgnm;
    float dhnm;
} wmm_cof_record_t;

void wmm_init(void);
float wmm_get_date(uint8_t year, uint8_t month, uint8_t date);
void E0000(float glat, float glon, float time_years, float *dec);

#endif
