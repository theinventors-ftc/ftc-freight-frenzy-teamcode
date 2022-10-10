package org.firstinspires.ftc.teamcode;

public class Button {
    public String state;

    public Button() {
        this.state = "released";
    }

    public boolean is_pressed() {
        return this.state == "pressed";
    }

    public boolean is_bumped() {
        return this.state == "bumped";
    }

    public String get_state() {
        return this.state;
    }

    public void update(boolean button_value) {
        if (!this.is_pressed() && button_value)
            this.state = "pressed";
        else if (this.is_pressed() && !button_value)
            this.state = "bumped";
        else if (!button_value)
            this.state = "released";
    }
}
