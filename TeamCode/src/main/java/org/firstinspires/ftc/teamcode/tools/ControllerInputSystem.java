package org.firstinspires.ftc.teamcode.tools;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ControllerInputSystem {
    private List<String> pressedButtons;

    private final String[] faceButtons = {"A", "B", "X", "Y", "Dpad Up", "Dpad Down", "Dpad Left", "Dpad Right", "Start", "Back"};

    Gamepad gamepad;

    public ControllerInputSystem(Gamepad gamepad){
        pressedButtons = new ArrayList<>();

        this.gamepad = gamepad;
    }

    public void updatePressedButtons() {
        // Check each button and add it to the list if pressed
        checkButton(gamepad.a, "A");
        checkButton(gamepad.b, "B");
        checkButton(gamepad.x, "X");
        checkButton(gamepad.y, "Y");
        checkButton(gamepad.dpad_up, "Dpad Up");
        checkButton(gamepad.dpad_down, "Dpad Down");
        checkButton(gamepad.dpad_left, "Dpad Left");
        checkButton(gamepad.dpad_right, "Dpad Right");
        checkButton(gamepad.left_bumper, "Left Bumper");
        checkButton(gamepad.right_bumper, "Right Bumper");
        checkButton(gamepad.left_trigger > 0.5, "Left Trigger");
        checkButton(gamepad.right_trigger > 0.5, "Right Trigger");
        checkButton(gamepad.start, "Start");
        checkButton(gamepad.back, "Back");
    }

    // Check if a button is pressed or released and update the list accordingly
    private void checkButton(boolean isPressed, String buttonName) {
        if (isPressed && !pressedButtons.contains(buttonName)) {
            // Add button to list if it's pressed and not already in the list
            pressedButtons.add(buttonName);
        } else if (!isPressed && pressedButtons.contains(buttonName)) {
            // Remove button from list if it's released
            pressedButtons.remove(buttonName);
        }
    }

    // Get the list of currently pressed buttons
    public List<String> getPressedButtons() {
        return pressedButtons;
    }

    public List<String> getPressedFaceButtons(){
        List<String> output = new ArrayList<>();

        for(String s : pressedButtons){
            if(Arrays.asList(faceButtons).contains(s)){
                output.add(s);
            }
        }

        return output;
    }

    @NonNull
    public String toString(){
        StringBuilder output = new StringBuilder();

        for(String s : pressedButtons){
            output.append(s);
            output.append(" ");
        }

        return output.toString();
    }
}
