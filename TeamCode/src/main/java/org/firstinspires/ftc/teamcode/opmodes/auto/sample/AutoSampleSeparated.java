package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.PICKUP1;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.PICKUP2;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.PICKUP3;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.RobotPosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.SCORE;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.START;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.LowerUpperTransferSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.CameraUpdateDetectorResult;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTThetaR;
import org.firstinspires.ftc.teamcode.opmodes.auto.SubmersibleSelectionGUI;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.RobotStateStore;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@Autonomous(name = "A. Sample Cycle Auto Separated", preselectTeleOp = "A. Teleop")
public final class AutoSampleSeparated extends LinearOpMode {
    private MecanumDrive drive;
    private LowerSlide lowSlide;
    private LowerSlideCommands lowerSlideCommands;
    private UpperSlide upSlide;
    private UpperSlideCommands upperSlideCommands;
    private BulkReadManager bulkReadManager;
    private Limelight camera;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        bulkReadManager = new BulkReadManager(hardwareMap);
        lowSlide = new LowerSlide();
        upSlide = new UpperSlide();
        lowSlide.initialize(hardwareMap);
        upSlide.initialize(hardwareMap);

        // Initialize command factories
        lowerSlideCommands = new LowerSlideCommands(lowSlide);
        upperSlideCommands = new UpperSlideCommands(upSlide);

        // Initialize camera
        camera = new Limelight();
        camera.initialize(hardwareMap);
        camera.cameraStart();

        // Initialize drive with starting pose
        drive = new MecanumDrive(hardwareMap, START.pose);

        // Start position - prepare slides
        Actions.runBlocking(new LowerSlideUpdatePID(lowSlide).toAction());
        Actions.runBlocking(new UpperSlideUpdatePID(upSlide).toAction());
        Actions.runBlocking(lowerSlideCommands.up());
        Actions.runBlocking(upperSlideCommands.scorespec());
        Actions.runBlocking(upperSlideCommands.closeClaw());

        // Color selection
        String color = "blue"; // Default color
        ElapsedTime elapsedTime = new ElapsedTime();
        boolean colorSelected = false;
        SubmersibleSelectionGUI gui = new SubmersibleSelectionGUI();

        while (!isStarted() && !isStopRequested()) {
            // Color selection
            while (!colorSelected) {
                telemetry.addLine("Use D-Pad Right to change color for sample detection, A to go to next phase");
                telemetry.addData("selected color: ", color);
                telemetry.update();

                if (0.25 < elapsedTime.seconds() && gamepad1.dpad_right) {
                    elapsedTime.reset();
                    color = color.equals("red") ? "blue" : "red";
                }

                if(gamepad1.a) {
                    colorSelected = true;
                }
            }

            // GUI selection
            gui.drawSub(gamepad1, telemetry);
            telemetry.update();
            sleep(50);
        }

        // Set Limelight color based on selection
        if (color.equals("blue")) {
            camera.setAcceptedColors(true, false, true); // blue, not red, yellow
        } else if (color.equals("red")) {
            camera.setAcceptedColors(false, true, true); // not blue, red, yellow
        } else {
            camera.setAcceptedColors(false, false, true); // fallback: yellow
        }

        ArrayList<Pose2d> selectedPickupPoints = gui.getDriverSelect();
        boolean useFallback = selectedPickupPoints.isEmpty();
        List<Vector2d> pickupPoints;
        if (useFallback) {
            pickupPoints = List.of(
                    new Vector2d(39, 28),
                    new Vector2d(39, 0));
        } else {
            pickupPoints = selectedPickupPoints.stream()
                    .map(p -> new Vector2d(p.position.x + 20, p.position.y))
                    .collect(Collectors.toList());
        }

        waitForStart();
        if (isStopRequested()) return;

        // Main autonomous sequence - separated into individual runBlocking actions

        // 1. Initial setup and first pickup preparation
        Actions.runBlocking(upperSlideCommands.scorespec());
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(START.pose)
                        .strafeToSplineHeading(PICKUP1.pos, PICKUP1.heading)
                        .build(),
                upperSlideCommands.closeClaw(),
                upperSlideCommands.front(),
                upperSlideCommands.slidePos3(),
                lowerSlideCommands.setSlidePos(ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST),
                lowerSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO)
        ));

        // 2. Score initial sample
        Actions.runBlocking(dropAndResetUpperSlides());

        // 3. First pickup sequence
        Actions.runBlocking(new CameraUpdateDetectorResult(camera).toAction());
        Actions.runBlocking(new DistanceAdjustLUTThetaR(lowSlide, drive,
                camera::getTx, camera::getTy, camera::getPx, camera::getPy,
                () -> {}, () -> {}).toAction());
        Actions.runBlocking(new LowerSlideGrabSequenceCommand(lowSlide).toAction());

        // 4. Drive to second pickup while transferring
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToSplineHeading(PICKUP2.pos, PICKUP2.heading)
                        .build(),
                new SequentialAction(
                        new LowerUpperTransferSequenceCommand(lowerSlideCommands, upperSlideCommands).toAction(),
                        new WaitCommand(ConfigVariables.AutoTesting.X_TRANSFERWHILEDRIVEAFTERTRANSFERDELAY_S).toAction(),
                        upperSlideCommands.inter()
                )
        ));

        // 5. Score first pickup
        Actions.runBlocking(upperSlideCommands.slidePos3());
        Actions.runBlocking(upperSlideCommands.front());
        Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction());
        Actions.runBlocking(upperSlideCommands.openClaw());
        Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S).toAction());
        Actions.runBlocking(new ParallelAction(
                upperSlideCommands.scorespec(),
                upperSlideCommands.slidePos0()
        ));

        // 6. Prepare for second pickup
        Actions.runBlocking(new ParallelAction(
                lowerSlideCommands.setSlidePos(ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND),
                lowerSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 45)
        ));
        Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.Y_PICKUPDELAY).toAction());

        // 7. Second pickup sequence
        Actions.runBlocking(new CameraUpdateDetectorResult(camera).toAction());
        Actions.runBlocking(new DistanceAdjustLUTThetaR(lowSlide, drive,
                camera::getTx, camera::getTy, camera::getPx, camera::getPy,
                () -> {}, () -> {}).toAction());
        Actions.runBlocking(new LowerSlideGrabSequenceCommand(lowSlide).toAction());

        // 8. Drive to score position while transferring
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToSplineHeading(SCORE.pos, SCORE.heading)
                        .build(),
                new SequentialAction(
                        new LowerUpperTransferSequenceCommand(lowerSlideCommands, upperSlideCommands).toAction(),
                        new WaitCommand(ConfigVariables.AutoTesting.X_TRANSFERWHILEDRIVEAFTERTRANSFERDELAY_S).toAction(),
                        upperSlideCommands.inter(),
                        upperSlideCommands.slidePos3()
                )
        ));

        // 9. Score second pickup
        Actions.runBlocking(upperSlideCommands.front());
        Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction());
        Actions.runBlocking(upperSlideCommands.openClaw());
        Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S).toAction());
        Actions.runBlocking(new ParallelAction(
                upperSlideCommands.scorespec(),
                upperSlideCommands.slidePos0()
        ));

        // 10. Third pickup sequence
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToSplineHeading(PICKUP3.pos, PICKUP3.heading)
                        .build(),
                lowerSlideCommands.setSlidePos(ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD),
                lowerSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 90)
        ));

        Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.Y_PICKUPDELAY).toAction());
        Actions.runBlocking(new CameraUpdateDetectorResult(camera).toAction());
        Actions.runBlocking(new DistanceAdjustLUTThetaR(lowSlide, drive,
                camera::getTx, camera::getTy, camera::getPx, camera::getPy,
                () -> {}, () -> {}).toAction());
        Actions.runBlocking(new LowerSlideGrabSequenceCommand(lowSlide).toAction());

        // 11. Drive to score position while transferring
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToSplineHeading(SCORE.pos, SCORE.heading)
                        .build(),
                new SequentialAction(
                        new LowerUpperTransferSequenceCommand(lowerSlideCommands, upperSlideCommands).toAction(),
                        new WaitCommand(ConfigVariables.AutoTesting.X_TRANSFERWHILEDRIVEAFTERTRANSFERDELAY_S).toAction(),
                        upperSlideCommands.inter(),
                        upperSlideCommands.slidePos3()
                )
        ));

        // 12. Score third pickup
        Actions.runBlocking(upperSlideCommands.front());
        Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction());
        Actions.runBlocking(upperSlideCommands.openClaw());
        Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S).toAction());
        Actions.runBlocking(new ParallelAction(
                upperSlideCommands.scorespec(),
                upperSlideCommands.slidePos0()
        ));

        // Additional pickup cycles from GUI selection
        for (Vector2d pickupVec : pickupPoints) {
            // Drive to pickup position
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(39, 28))
                            .splineTo(pickupVec, Math.toRadians(-160))
                            .build(),
                    lowerSlideCommands.slidePos1()
            ));

            Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.I_SUBDELAY_S).toAction());

            // Adjust and pickup
            Actions.runBlocking(new CameraUpdateDetectorResult(camera).toAction());
            Actions.runBlocking(new WaitCommand(ConfigVariables.Camera.CAMERA_DELAY).toAction());
            Actions.runBlocking(new AngleAdjustCommand(lowSlide, camera).toAction());
            Actions.runBlocking(new LowerSlideGrabSequenceCommand(lowSlide).toAction());

            // Drive back to score while transferring
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            .setReversed(true)
                            .splineTo(new Vector2d(44, 28), SCORE.heading - Math.toRadians(170))
                            .splineTo(SCORE.pos, SCORE.heading - Math.toRadians(170))
                            .build(),
                    new SequentialAction(
                            new WaitCommand(ConfigVariables.AutoTesting.K_ROUNDPATHEXITTIME_S).toAction(),
                            new LowerUpperTransferSequenceCommand(lowerSlideCommands, upperSlideCommands).toAction(),
                            new WaitCommand(ConfigVariables.AutoTesting.X_TRANSFERWHILEDRIVEAFTERTRANSFERDELAY_S).toAction(),
                            upperSlideCommands.inter(),
                            upperSlideCommands.slidePos3()
                    )
            ));

            // Score
            Actions.runBlocking(upperSlideCommands.front());
            Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction());
            Actions.runBlocking(upperSlideCommands.openClaw());
            Actions.runBlocking(new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S).toAction());
            Actions.runBlocking(new ParallelAction(
                    upperSlideCommands.scorespec(),
                    upperSlideCommands.slidePos0()
            ));
        }

        // Save final robot state
        RobotStateStore.save(drive.localizer.getPose(), lowSlide.getCurrentPosition(), upSlide.getCurrentPosition());
    }

    // Helper method for drop and reset sequence
    private SequentialAction dropAndResetUpperSlides() {
        return new SequentialAction(
                upperSlideCommands.openClaw(),
                new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S).toAction(),
                new ParallelAction(
                        upperSlideCommands.scorespec(),
                        upperSlideCommands.slidePos0()
                )
        );
    }
}