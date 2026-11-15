
package org.firstinspires.ftc.teamcode.utils;

/**
 * Defines the ball types used by the FTC flywheel subsystem.
 *
 * Big   = Nectar ball
 * Small = Pollen ball
 * None  = No ball selected
 */
public enum BallType {

    Big,
    Small,
    None;

    /**
     * Returns a readable name for the ball type.
     */
    public String getDisplayName() {
        switch (this) {
            case Big:
                return "Nectar";
            case Small:
                return "Pollen";
            case None:
            default:
                return "No Ball Selected";
        }
    }

    /**
     * Returns whether a valid ball type has been selected.
     */
    public boolean isValid() {
        return this != None;
    }

    /**
     * Returns the corresponding ball type from a string.
     */
    public static BallType fromString(String value) {
        if (value == null) {
            return None;
        }

        switch (value.trim().toLowerCase()) {
            case "big":
            case "nectar":
                return Big;

            case "small":
            case "pollen":
                return Small;

            default:
                return None;
        }
    }
}
