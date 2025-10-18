package org.firstinspires.ftc.teamcode.teleop.utils;

public class Toggle {
    public boolean value;

    // If the button is being held, the toggle is locked from changing until the button is let go
    private boolean buttonLock = false;

    public Toggle(boolean initialValue) { this.value = initialValue; }

    public void update(boolean buttonPressed) {
        // Returns true if value has been toggled, false otherwise
        if (buttonPressed) {
            if (!buttonLock) {
                value = !value;
                buttonLock = true;
                return;
            }
        } else { buttonLock = false; }
    }

    public void set(boolean value) { this.value = value; }
    public boolean value() { return value; }
    public boolean locked() { return buttonLock; }
}
