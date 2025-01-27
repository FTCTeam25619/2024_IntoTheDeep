package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.teamcode.ConfigConstants;

public enum GamePieceColor {
    BLUE(ConfigConstants.Colors.BLUE_HUE_MIN, ConfigConstants.Colors.BLUE_HUE_MAX),
    RED(ConfigConstants.Colors.RED_HUE_MIN, ConfigConstants.Colors.RED_HUE_MAX),
    YELLOW(ConfigConstants.Colors.YELLOW_HUE_MIN, ConfigConstants.Colors.YELLOW_HUE_MAX),
    BLACK(ConfigConstants.Colors.BLACK_HUE_MIN, ConfigConstants.Colors.BLACK_HUE_MAX);

    public final float hueMin;
    public final float hueMax;

    private GamePieceColor(float hueMin, float hueMax) {
        this.hueMin = hueMin;
        this.hueMax = hueMax;
    }

    public boolean matches(float hue) {
        return hue >= hueMin && hue <= hueMax;
    }
}
