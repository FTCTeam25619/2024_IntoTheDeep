package org.firstinspires.ftc.teamcode.lib;

import android.graphics.Color;
import android.graphics.ColorSpace;

import java.lang.Math;

public class ColorUtils {
    Color color;

    static double[] argbToLab(int alpha, int red, int green, int blue) {
        /*
         * Steps:
         * 1. Normalize RGB channels
         * 2. Convert to XYZ color space
         * 3. Normalize XYZ channels
         * 4. Convert to Lab color space
         */
        double red_norm, green_norm, blue_norm, X, Y, Z, L, a, b;

        // 1. Normalize RGB channels
        red_norm = normalizeRGBChannel(red);
        green_norm = normalizeRGBChannel(green);
        blue_norm = normalizeRGBChannel(blue);

        // 2. Convert to XYZ color space with Observer= 2°, Illuminant= D65
        final double refX = 95.047, refY = 100.000, refZ = 108.883;
        X = red_norm * 0.4124 + green_norm * 0.3576 + blue_norm * 0.1805;
        Y = red_norm * 0.2126 + green_norm * 0.7152 + blue_norm * 0.0722;
        Z = red_norm * 0.0193 + green_norm * 0.1192 + blue_norm * 0.9505;
        X = roundToPlaces(X, 4) / refX;
        Y = roundToPlaces(Y, 4) / refY;
        Z = roundToPlaces(Z, 4) / refZ;

        // 3. Normalize XYZ channels
        X = normalizeXYZChannel(X);
        Y = normalizeXYZChannel(Y);
        Z = normalizeXYZChannel(Z);

        // 4. Convert to Lab color space
        L = roundToPlaces((116.0 * Y) - 16.0, 4);
        a = roundToPlaces(500.0 * (X - Y), 4);
        b = roundToPlaces(200.0 * (Y - Z), 4);

        return new double[]{L, a, b, alpha};
    }

    static double ciede2000(final double l_1, final double a_1, final double b_1, final double l_2, final double a_2, final double b_2) {
        // Working with the CIEDE2000 color-difference formula.
        // k_l, k_c, k_h are parametric factors to be adjusted according to
        // different viewing parameters such as textures, backgrounds...
        final double k_l = 1.0, k_c = 1.0, k_h = 1.0;
        double n = (Math.hypot(a_1, b_1) + Math.hypot(a_2, b_2)) * 0.5;
        n = n * n * n * n * n * n * n;
        // A factor involving chroma raised to the power of 7 designed to make
        // the influence of chroma on the total color difference more accurate.
        n = 1.0 + 0.5 * (1.0 - Math.sqrt(n / (n + 6103515625.0)));
        // hypot calculates the Euclidean distance while avoiding overflow/underflow.
        final double c_1 = Math.hypot(a_1 * n, b_1), c_2 = Math.hypot(a_2 * n, b_2);
        // atan2 is preferred over atan because it accurately computes the angle of
        // a point (x, y) in all quadrants, handling the signs of both coordinates.
        double h_1 = Math.atan2(b_1, a_1 * n), h_2 = Math.atan2(b_2, a_2 * n);
        h_1 += 2.0 * Math.PI * Boolean.compare(h_1 < 0.0, false);
        h_2 += 2.0 * Math.PI * Boolean.compare(h_2 < 0.0, false);
        n = Math.abs(h_2 - h_1);
        // Cross-implementation consistent rounding.
        if (Math.PI - 1E-14 < n && n < Math.PI + 1E-14)
            n = Math.PI;
        // When the hue angles lie in different quadrants, the straightforward
        // average can produce a mean that incorrectly suggests a hue angle in
        // the wrong quadrant, the next lines handle this issue.
        double h_m = 0.5 * h_1 + 0.5 * h_2, h_d = (h_2 - h_1) * 0.5;
        if (Math.PI < n) {
            if (0.0 < h_d)
                h_d -= Math.PI;
            else
                h_d += Math.PI;
            h_m += Math.PI;
        }
        final double p = (36.0 * h_m - 55.0 * Math.PI);
        n = (c_1 + c_2) * 0.5;
        n = n * n * n * n * n * n * n;
        // The hue rotation correction term is designed to account for the
        // non-linear behavior of hue differences in the blue region.
        final double r_t = -2.0 * Math.sqrt(n / (n + 6103515625.0))
                * Math.sin(Math.PI / 3.0 * Math.exp(p * p / (-25.0 * Math.PI * Math.PI)));
        n = (l_1 + l_2) * 0.5;
        n = (n - 50.0) * (n - 50.0);
        // Lightness.
        final double l = (l_2 - l_1) / (k_l * (1.0 + 0.015 * n / Math.sqrt(20.0 + n)));
        // These coefficients adjust the impact of different harmonic
        // components on the hue difference calculation.
        final double t = 1.0 + 0.24 * Math.sin(2.0 * h_m + Math.PI / 2)
                + 0.32 * Math.sin(3.0 * h_m + 8.0 * Math.PI / 15.0)
                - 0.17 * Math.sin(h_m + Math.PI / 3.0)
                - 0.20 * Math.sin(4.0 * h_m + 3.0 * Math.PI / 20.0);
        n = c_1 + c_2;
        // Hue.
        final double h = 2.0 * Math.sqrt(c_1 * c_2) * Math.sin(h_d) / (k_h * (1.0 + 0.0075 * n * t));
        // Chroma.
        final double c = (c_2 - c_1) / (k_c * (1.0 + 0.0225 * n));
        // Returning the square root ensures that the result represents
        // the "true" geometric distance in the color space.
        return Math.sqrt(l * l + h * h + c * c + c * h * r_t);
    }

    private static double roundToPlaces(double value, int places) {
        double scale = Math.pow(10.0, places);
        return (double)Math.round(value * scale) / scale;
    }

    private static double normalizeRGBChannel(int channel) {
        double value = (double)channel / 255.0;
        return value > 0.04045 ? Math.pow(value + 0.055, 2.4) : value / 12.92;
    }

    private static double normalizeXYZChannel(double channel) {
        return channel > 0.008856 ? Math.pow(channel, 1.0/3.0) : (7.787 * channel) / (16.0/116.0);
    }
}
