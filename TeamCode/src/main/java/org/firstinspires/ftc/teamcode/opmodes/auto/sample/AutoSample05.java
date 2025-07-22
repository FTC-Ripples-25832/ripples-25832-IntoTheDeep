package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.PICKUP1;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.PICKUP2;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.PICKUP3;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.RobotPosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.SCORE;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.START;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerUpperTransferSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
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
import org.firstinspires.ftc.teamcode.utils.timing.Interval;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@Autonomous(name = "A. Sample Cycle Auto 05", preselectTeleOp = "A. Teleop")
public final class AutoSample05 extends LinearOpMode {

    private MecanumDrive drive;
    private LowerSlide lowSlide;
    private LowerSlideCommands lowerSlideCommands;
    private UpperSlide upSlide;
    private UpperSlideCommands upperSlideCommands;
    private BulkReadManager bulkReadManager;
    private Limelight camera;

    private ParallelAction prepUpperSlides() {
        return new ParallelAction(
                upperSlideCommands.closeClaw(),
                upperSlideCommands.front(),
                upperSlideCommands.slidePos3()
        );
    }

    private Action driveToScore(RobotPosition startPOS, RobotPosition scorePOS) {
        return drive.actionBuilder(startPOS.pose)
                .strafeToSplineHeading(scorePOS.pos, scorePOS.heading)
                .build();
    }

    private Action transferSequence() {
        return new LowerUpperTransferSequenceCommand(lowerSlideCommands, upperSlideCommands).toAction();
    }

    private Action pickupSequence() {
        return new LowerSlideGrabSequenceCommand(lowSlide).toAction();
    }

    private SequentialAction adjustSequence() {
        return new SequentialAction(
                new CameraUpdateDetectorResult(camera).toAction(),
                new DistanceAdjustLUTThetaR(lowSlide, drive,
                        camera::getTx, camera::getTy, camera::getPx, camera::getPy,
                        () -> {
                        }, () -> {
                }).toAction()
        );
    }

    private SequentialAction dropAndResetUpperSlides() {
        return new SequentialAction(
                upperSlideCommands.openClaw(),
                new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S).toAction(),
                upperSlideCommands.scorespec(),
                upperSlideCommands.slidePos0()
        );
    }

    private SequentialAction frontForDrop() {
        return new SequentialAction(
                upperSlideCommands.front(),
                new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction()
        );
    }

    private SequentialAction transferWhileDriving() {
        return new SequentialAction(
                transferSequence(),
                new WaitCommand(ConfigVariables.AutoTesting.X_TRANSFERWHILEDRIVEAFTERTRANSFERDELAY_S).toAction(),
                upperSlideCommands.inter(),
                upperSlideCommands.slidePos3()
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bulkReadManager = new BulkReadManager(hardwareMap);
        lowSlide = new LowerSlide();
        upSlide = new UpperSlide();
        lowSlide.initialize(hardwareMap);
        upSlide.initialize(hardwareMap);
        lowerSlideCommands = new LowerSlideCommands(lowSlide);
        upperSlideCommands = new UpperSlideCommands(upSlide);
        camera = new Limelight();
        camera.initialize(hardwareMap);
        camera.cameraStart();

        drive = new MecanumDrive(hardwareMap, START.pose);

        Actions.runBlocking(lowerSlideCommands.up());
        Actions.runBlocking(upperSlideCommands.scorespec());
        Actions.runBlocking(upperSlideCommands.closeClaw());

        SubmersibleSelectionGUI gui = new SubmersibleSelectionGUI();
        String color = "blue";

        while (!isStarted() && !isStopRequested()) {
            gui.drawSub(gamepad1, telemetry);
            sleep(50);
        }

        if (color.equals("blue")) {
            camera.setAcceptedColors(true, false, true);
        } else if (color.equals("red")) {
            camera.setAcceptedColors(false, true, true);
        } else {
            camera.setAcceptedColors(false, false, true);
        }

        ArrayList<Pose2d> selectedPickupPoints = gui.getDriverSelect();
        List<Vector2d> pickupPoints = selectedPickupPoints.isEmpty()
                ? List.of(new Vector2d(39, 28), new Vector2d(39, 0))
                : selectedPickupPoints.stream()
                .map(p -> new Vector2d(p.position.x + 20, p.position.y))
                .collect(Collectors.toList());

        waitForStart();
        if (isStopRequested()) return;

        new Interval(lowSlide::updatePID, 10);
        new Interval(upSlide::updatePID, 10);

        // === 分步执行 ===

        // 1) START → PICKUP1
        Actions.runBlocking(driveToScore(START, PICKUP1));

        // 2) 上滑轨准备
        Actions.runBlocking(prepUpperSlides());

        // 3) 放下 + 复位
        Actions.runBlocking(dropAndResetUpperSlides());

        // 4) 当前 → PICKUP2，同时 Transfer
        Actions.runBlocking(new ParallelAction(
                driveToScore(PICKUP1, PICKUP2),
                transferWhileDriving()
        ));

        // 5) Front for Drop
        Actions.runBlocking(frontForDrop());
        Actions.runBlocking(dropAndResetUpperSlides());

        // 6) 当前 → SCORE
        Actions.runBlocking(driveToScore(PICKUP2, SCORE));
        Actions.runBlocking(frontForDrop());
        Actions.runBlocking(dropAndResetUpperSlides());

        // 7) 当前 → PICKUP3
        Actions.runBlocking(driveToScore(SCORE, PICKUP3));
        Actions.runBlocking(lowerSlideCommands.setSlidePos(ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD));
        Actions.runBlocking(adjustSequence());
        Actions.runBlocking(pickupSequence());

        Actions.runBlocking(new ParallelAction(
                driveToScore(PICKUP3, SCORE),
                transferWhileDriving()
        ));
        Actions.runBlocking(frontForDrop());
        Actions.runBlocking(dropAndResetUpperSlides());

        // 8) 自定义 PickupPoints 循环
        for (Vector2d pickupVec : pickupPoints) {
            Actions.runBlocking(drive.actionBuilder(SCORE.pose)
                    .strafeTo(new Vector2d(39, 28))
                    .splineTo(pickupVec, Math.toRadians(-160))
                    .build());

            Actions.runBlocking(adjustSequence());
            Actions.runBlocking(new AngleAdjustCommand(lowSlide, camera).toAction());
            Actions.runBlocking(pickupSequence());

            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToSplineHeading(new Vector2d(44, 28), SCORE.heading)
                            .setReversed(true)
                            .splineTo(SCORE.pos, SCORE.heading - Math.toRadians(170))
                            .build(),
                    transferWhileDriving()
            ));

            Actions.runBlocking(frontForDrop());
            Actions.runBlocking(dropAndResetUpperSlides());
        }

        RobotStateStore.save(drive.localizer.getPose(), lowSlide.getCurrentPosition(), upSlide.getCurrentPosition());
    }
}
