package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.concurrent.Callable;

public class Button {

    private Gamepad gamepad;
    private GamepadKeys.ButtonType button;

    private boolean old = false;


    Button(Gamepad gamepad, GamepadKeys.ButtonType button) {
        this.button = button;
        this.gamepad = gamepad;
    }

    public void on_pressed( Callable func ) {

        if (button_type_to_bool(button) && !old) {
            old = true;

            try {
                func.call();
            } catch (Exception e) { }

        } else if (!button_type_to_bool(button) && old) {
            old = false;
        }

    }

    public void on_release( Callable func) {

        if (button_type_to_bool(button) && !old) {
            old = true;

        } else if (!button_type_to_bool(button) && old) {

            try {
                func.call();
            } catch (Exception e) { }

            old = false;
        }
    }

    private boolean button_type_to_bool (GamepadKeys.ButtonType button) {
        switch (button) {
            case X:
                return gamepad.x;
            default:
                return false;
        }
    }
}
