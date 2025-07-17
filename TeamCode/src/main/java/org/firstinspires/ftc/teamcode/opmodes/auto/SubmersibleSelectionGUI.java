package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Rotation2d;
import org.firstinspires.ftc.teamcode.utils.interfaces.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.interfaces.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import java.util.ArrayList;

public class SubmersibleSelectionGUI {
        // Submersible 45in by 28in
        private final double length = 27.5, width = 44.5;
        private final int asciiLen = 19, asciiWidth = 15;
        private final boolean[][] sub = new boolean[asciiLen][asciiWidth];

        private int cursorX = asciiLen / 2, cursorY = asciiWidth / 2;

        public SubmersibleSelectionGUI() {
                // preselect();
        }

        public ArrayList<Pose2d> getDriverSelect() {
                ArrayList<Pose2d> coords = new ArrayList<>();

                for (int i = 0; i < asciiLen; i++) {
                        for (int j = 0; j < asciiWidth; j++) {
                                if (sub[i][j]) {
                                        coords.add(new Pose2d(
                                                        new Vector2d(
                                                                        (i + 0.5 - (double) asciiLen / 2.0) * length
                                                                                        / (double) asciiLen,
                                                                        (j + 0.5 - (double) asciiWidth / 2.0) * width
                                                                                        / (double) asciiWidth),
                                                        0.0 // heading
                                        ));
                                }
                        }
                }

                return coords;
        }

        private boolean flash = true;
        private long timer = System.currentTimeMillis();
        private final ButtonToggle down = new ButtonToggle(), up = new ButtonToggle(), left = new ButtonToggle(),
                        right = new ButtonToggle(), place = new ButtonToggle();
        private final ButtonToggle preset = new ButtonToggle();

        public void drawSub(Gamepad gamepad, Telemetry tele) {
                for (int j = 0; j < asciiWidth; j++) {
                        StringBuilder curr = new StringBuilder();
                        for (int i = asciiLen - 1; i >= 0; i--) {
                                if (flash && i == cursorX && j == cursorY) {
                                        curr.append("x");
                                } else {
                                        curr.append(sub[i][j] ? "#" : "_");
                                }
                        }

                        tele.addData((j >= 10 ? "" : "0") + j, curr.toString());
                }

                Canvas canvas = TelemetryUtil.packet.fieldOverlay();
                canvas.setFill("#ff80ff");
                for (Pose2d blockPos : getDriverSelect()) {
                        canvas.fillCircle(blockPos.position.x, blockPos.position.y, 1);
                }

                tele.update();
                // TelemetryUtil.sendTelemetry();

                if (down.isClicked(gamepad.dpad_down || gamepad.right_stick_y > 0.2)) {
                        cursorY = Utils.minMaxClip(cursorY + 1, 0, asciiWidth - 1);
                        timer = System.currentTimeMillis();
                        flash = true;
                }
                if (up.isClicked(gamepad.dpad_up || gamepad.right_stick_y < -0.2)) {
                        cursorY = Utils.minMaxClip(cursorY - 1, 0, asciiWidth - 1);
                        timer = System.currentTimeMillis();
                        flash = true;
                }
                if (left.isClicked(gamepad.dpad_left || gamepad.right_stick_x < -0.2)) {
                        cursorX = Utils.minMaxClip(cursorX + 1, 0, asciiLen - 1);
                        timer = System.currentTimeMillis();
                        flash = true;
                }
                if (right.isClicked(gamepad.dpad_right || gamepad.right_stick_x > 0.2)) {
                        cursorX = Utils.minMaxClip(cursorX - 1, 0, asciiLen - 1);
                        timer = System.currentTimeMillis();
                        flash = true;
                }

                if (place.isClicked(gamepad.a || gamepad.left_bumper)) {
                        sub[cursorX][cursorY] = !sub[cursorX][cursorY];
                }

                if (System.currentTimeMillis() - timer >= 500) {
                        timer = System.currentTimeMillis();
                        flash = !flash;
                }

                if (preset.isClicked(gamepad.y))
                        preselect();
        }

        private void preselect() {
                for (int i = 12; i <= 17; ++i)
                        for (int j = i % 2 == 0 ? 8 : 9; j <= 12; j += 2)
                                sub[i][j] = true;
        }
}
