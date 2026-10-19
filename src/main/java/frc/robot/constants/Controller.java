package frc.robot.constants;

public final class Controller {
    /** Whether drive is based off of the field (bird's eye) or the robot's heading (robot POV) */
    public static final boolean IS_HEADING_BASED_DRIVE = false;

    /** Area in which joystick does not register. Prevents twitching when staying still. */
    public static final double HEADING_DEADZONE = 0.2;

    // -- Port (identifier) for each individual joystick/controller plugged into computer.
    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int MIDDLE_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 2;

    /** A placeholder value to signify that it is unbound */
    public static final int UNBOUND = 0;

    /* -- Individual mappings for buttons on each joystick, from 1-16. Note that right-handed
     * -- controllers and left-handed controllers have opposite and reflected bindings (see below).
     */
    public static final int TRIGGER = 1;
    public static final int THUMB_BUTTON_BOTTOM = 2;
    public static final int THUMB_BUTTON_LEFT = 3;
    public static final int THUMB_BUTTON_RIGHT = 4;

    /* Each joystick is either left or right handed. When holding the joystick, the buttons on
     * the side of the opposing hand are the offhand buttons. The buttons on the side of the hand
     * holding the joystick is the mainhand buttons.
     */
    public static final int OFFHAND_TOP_LEFT = 5;
    public static final int OFFHAND_TOP_MIDDLE = 6;
    public static final int OFFHAND_TOP_RIGHT = 7;
    public static final int OFFHAND_BOTTOM_RIGHT = 8;
    public static final int OFFHAND_BOTTOM_MIDDLE = 9;
    public static final int OFFHAND_BOTTOM_LEFT = 10;

    public static final int MAINHAND_TOP_RIGHT = 11;
    public static final int MAINHAND_TOP_MIDDLE = 12;
    public static final int MAINHAND_TOP_LEFT = 13;
    public static final int MAINHAND_BOTTOM_LEFT = 14;
    public static final int MAINHAND_BOTTOM_MIDDLE = 15;
    public static final int MAINHAND_BOTTOM_RIGHT = 16;
}
