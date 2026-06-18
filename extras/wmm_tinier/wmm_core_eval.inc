void E0000(float glat, float glon, float time_years, float *dec)
{
    static float tc[13][13];
    static float sp[13];
    static float cp[13];
    static float dp[13][13];
    static float pp[13];
    float dt = time_years - WMM_EPOCH;
    float rlon = glon * DEGREES_TO_RADIANS;
    float rlat = glat * DEGREES_TO_RADIANS;
    float srlon = sinf(rlon);
    float srlat = sinf(rlat);
    float crlon = cosf(rlon);
    float crlat = cosf(rlat);
    float srlat2 = srlat * srlat;
    float crlat2 = crlat * crlat;
    sp[0] = 0.0f;
    sp[1] = srlon;
    cp[0] = 1.0f;
    cp[1] = crlon;
    dp[0][0] = 0.0f;
    pp[0] = 1.0f;

    float q = sqrtf(A2_CONST - C2_CONST * srlat2);
    float q2 = (A2_CONST / B2_CONST) * (A2_CONST / B2_CONST);
    float ct = srlat / sqrtf(q2 * crlat2 + srlat2);
    float st = sqrtf(1.0f - (ct * ct));
    float r2 = (A4_CONST - C4_CONST * srlat2) / (q * q);
    float r = sqrtf(r2);
    float d = sqrtf(A2_CONST * crlat2 + B2_CONST * srlat2);
    float ca = d / r;
    float sa = C2_CONST * crlat * srlat / (r * d);
    for (uint8_t m = 2U; m <= 12U; m++) {
        sp[m] = sp[1] * cp[m - 1U] + cp[1] * sp[m - 1U];
        cp[m] = cp[1] * cp[m - 1U] - sp[1] * sp[m - 1U];
    }

    float aor = RE_CONST / r;
    float ar = aor * aor;
    float br = 0.0f;
    float bt = 0.0f;
    float bp = 0.0f;
    float bpp = 0.0f;

    for (uint16_t n = 1U; n <= 12U; n++) {
        ar = ar * aor;
        uint8_t m = 0U;
        for (uint8_t D4wmm = n + 1U; D4wmm > 0U; D4wmm--) {
            if (n == m) {
                *(snorm + n + m * 13U) = st * *(snorm + n - 1U + (m - 1U) * 13U);
                dp[m][n] = st * dp[m - 1U][n - 1U] + ct * *(snorm + n - 1U + (m - 1U) * 13U);
                goto S50;
            }
            if (n == 1U && m == 0U) {
                *(snorm + n + m * 13U) = ct * *(snorm + n - 1U + m * 13U);
                dp[m][n] = ct * dp[m][n - 1U] - st * *(snorm + n - 1U + m * 13U);
                goto S50;
            }
            if (n > 1U && n != m) {
                if (m > n - 2U) *(snorm + n - 2U + m * 13U) = 0.0f;
                if (m > n - 2U) dp[m][n - 2U] = 0.0f;
                *(snorm + n + m * 13U) = ct * *(snorm + n - 1U + m * 13U) - k[m][n] * *(snorm + n - 2U + m * 13U);
                dp[m][n] = ct * dp[m][n - 1U] - st * *(snorm + n - 1U + m * 13U) - k[m][n] * dp[m][n - 2U];
            }
        S50:
            tc[m][n] = c[m][n] + dt * cd[m][n];
            if (m != 0U) tc[n][m - 1U] = c[n][m - 1U] + dt * cd[n][m - 1U];

            float par = ar * *(snorm + n + m * 13U);
            float temp1;
            float temp2;
            if (m == 0U) {
                temp1 = tc[m][n] * cp[m];
                temp2 = tc[m][n] * sp[m];
            } else {
                temp1 = tc[m][n] * cp[m] + tc[n][m - 1U] * sp[m];
                temp2 = tc[m][n] * sp[m] - tc[n][m - 1U] * cp[m];
            }

            bt = bt - ar * temp1 * dp[m][n];
            bp += fm[m] * temp2 * par;
            br += fn[n] * temp1 * par;

            if (st == 0.0f && m == 1U) {
                if (n == 1U) pp[n] = pp[n - 1U];
                else pp[n] = ct * pp[n - 1U] - k[m][n] * pp[n - 2U];
                bpp += fm[m] * temp2 * ar * pp[n];
            }
            m += 1U;
        }
    }

    if (st == 0.0f) bp = bpp;
    else bp /= st;

    float bx = -bt * ca - br * sa;
    float by = bp;
    *dec = atan2f(by, bx) / DEGREES_TO_RADIANS;
}
