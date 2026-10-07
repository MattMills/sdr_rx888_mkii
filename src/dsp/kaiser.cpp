#include "kaiser.h"
#include <cmath>

namespace rx888 {

namespace {

constexpr double PI = 3.14159265358979323846;

// Zeroth order modified Bessel function of the first kind.
double izero(double x) {
    double x2 = x / 2.0;
    double sum = 1.0, ds = 1.0, di = 1.0;
    do {
        double t = x2 / di;
        ds *= t * t;
        sum += ds;
        di += 1.0;
    } while (ds >= 1e-12 * sum);
    return sum;
}

} // namespace

int kaiserLowpass(int numTaps, float astop, float normFpass, float normFstop, float* coef) {
    double fcut = (normFstop + normFpass) / 2.0; // 6 dB cutoff

    double beta;
    if (astop < 20.96f) { beta = 0.0; }
    else if (astop >= 50.0f) { beta = 0.1102 * (astop - 8.71); }
    else { beta = 0.5842 * pow(astop - 20.96, 0.4) + 0.07886 * (astop - 20.96); }

    int n = (int)((astop - 8.0) / (2.285 * 2.0 * PI * (normFstop - normFpass)) + 1);
    if (numTaps < 0 && n > -numTaps) { n = -numTaps; }
    if (n < 3) { n = 3; }
    if (numTaps <= 0 && !coef) { return n; }
    if (numTaps > 0) { n = numTaps; }

    double center = 0.5 * (n - 1);
    double izb = izero(beta);
    for (int i = 0; i < n; i++) {
        double x = i - center;
        double c = (x == 0.0) ? 2.0 * fcut : sin(2.0 * PI * x * fcut) / (PI * x);
        double w = (i - center) / center;
        coef[i] = (float)(c * izero(beta * sqrt(1.0 - w * w)) / izb);
    }
    return n;
}

} // namespace rx888
