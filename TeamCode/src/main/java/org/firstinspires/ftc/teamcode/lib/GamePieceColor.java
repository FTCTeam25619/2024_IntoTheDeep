package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.teamcode.Constants;

public enum GamePieceColor {
    BLUE(0, 0, 255),
    RED(255, 0, 0),
    YELLOW(100, 200, 50),
    BLACK(0, 0, 0);

    public final int red;
    public final int green;
    public final int blue;
    public final double[] Lab;

    private GamePieceColor(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.Lab = ColorUtils.argbToLab(255, red, green, blue);
    }

    public boolean matches(int[] argb) {
        return distance(this, argb) > Constants.Sensors.COLOR_MATCH_DISTANCE_THRESHOLD;
    }

    public static double distance(GamePieceColor color, int[] argb) {
        double[] testColorLab = ColorUtils.argbToLab(argb[0], argb[1], argb[2], argb[3]);
        return ColorUtils.ciede2000(
                testColorLab[0], testColorLab[1], testColorLab[2],
                color.Lab[0], color.Lab[1], color.Lab[2]);
    }
}
