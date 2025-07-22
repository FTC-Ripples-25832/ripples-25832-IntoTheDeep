package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.SCORE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.base.ConditionalCommand;
import org.firstinspires.ftc.teamcode.commands.base.LoopTimeTelemetryCommand;
import org.firstinspires.ftc.teamcode.commands.base.ReadRobotStateCommand;
import org.firstinspires.ftc.teamcode.commands.base.SaveRobotStateCommand;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.base.WaitForConditionCommand;
import org.firstinspires.ftc.teamcode.commands.drive.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.hang.HangingCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.LowerUpperTransferSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.MotionProfiledSlideCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideScoreCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.CameraUpdateDetectorResult;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTThetaR;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTX;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.ColorSensorImpl;
// import org.firstinspires.ftc.teamcode.sensors.limelight.LimeLightImageTools;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.ClawController;
import org.firstinspires.ftc.teamcode.utils.GamepadController;
import org.firstinspires.ftc.teamcode.utils.GamepadController.ButtonType;
import org.firstinspires.ftc.teamcode.utils.RobotStateStore;
import org.firstinspires.ftc.teamcode.utils.TelemetryPacket;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;

import java.util.List;

@TeleOp(group = "TeleOp", name = "A. Teleop")
public class Teleop extends LinearOpMode {

    private static List<LynxModule> allHubs = null;
    private final long starttime = System.currentTimeMillis();

    private MecanumDrive drive;

    private UpperSlide upSlide;
    private LowerSlide lowSlide;
    private Limelight camera;
    private Hanging hanging;

    private UpperSlideCommands upslideActions;
    private LowerSlideCommands lowslideActions;

    private CommandScheduler scheduler;
    private BulkReadManager bulkReadManager;

    private FtcDashboard dashboard;
    private long lastDashboardUpdateTime = 0;

    private GamepadController gamepad1Controller;
    private GamepadController gamepad2Controller;

    private ClawController upperClaw;
    private ClawController upperExtendo;
    private ClawController lowerClaw;

    private MecanumDriveCommand mecanumDriveCommand;

    private ColorSensorImpl colorSensor;
    private long loopCount = 0;
    private long lastloop = System.currentTimeMillis();

    @Override
    public void runOpMode() throws InterruptedException {

        scheduler = CommandScheduler.getInstance();

        initializeSubsystems();

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        mecanumDriveCommand = new MecanumDriveCommand(drive, gamepad1);
        scheduler.schedule(mecanumDriveCommand);
        // Schedule loop timing telemetry command
        scheduler.schedule(new LoopTimeTelemetryCommand());
        scheduler.schedule(new ActionCommand(upslideActions.front()));
        scheduler.schedule(new ActionCommand(lowslideActions.up()));

        if (ConfigVariables.General.WITH_STATESAVE) {
            RobotStateStore.loadSlides(lowSlide, upSlide);
        }
        while (!isStopRequested() && !opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);

            if(gamepad1.a){ // press gamepad1 A if lowarm stuck
                scheduler.schedule(new ActionCommand(lowslideActions.safeFloorToUp()));
            }

            if(gamepad2.a){ // press gamepad2 A if uparm stuck
                scheduler.schedule(new ActionCommand(upslideActions.safeBackToFront()));
            }
            // gamepad1Controller.update();
            // gamepad2Controller.update();
        }

        waitForStart();

        if (ConfigVariables.General.SINGLE_CONTROLLER_MODE) {
            setupGamepadControlsSingle();
        } else {
            setupGamepadControls();
        }
        setContinuousControls();

        if (isStopRequested())
            return;

        while (opModeIsActive() && !isStopRequested()) {
            loopCount += 1;
            long timestamp = System.currentTimeMillis();
            // PERFORMANCE OPTIMIZATION: Update bulk reads once per loop
            bulkReadManager.updateBulkRead();

            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);

            gamepad1Controller.update();
            gamepad2Controller.update();

            // update pid in command will cause power error
            lowSlide.updatePID();
            upSlide.updatePID();

            // log looptime for checking, when dashboard is not enabled
            if (loopCount % 20 == 0) {
                telemetry.addData("looptimems", (timestamp - lastloop));
                telemetry.addData("avglooptimems", (timestamp - starttime) / (double) loopCount);
                telemetry.update();
            }
            lastloop = timestamp;

            // only send packet if in debug mode
            if (ConfigVariables.General.DEBUG_MODE
                    && timestamp - lastDashboardUpdateTime >= ConfigVariables.General.DASHBOARD_UPDATE_INTERVAL_MS) {
                packet.put("gamepad1/NoOperationTimems", gamepad1Controller.getNoOperationTime());
                packet.put("gamepad2/NoOperationTimems", gamepad2Controller.getNoOperationTime());
                packet.put("colorsensor/caught", colorSensor.caught());
                packet.put("colorsensor/cantransfer", colorSensor.canTransfer());
                dashboard.sendTelemetryPacket(packet);
                lastDashboardUpdateTime = timestamp;
            }

        }
        if (ConfigVariables.General.WITH_STATESAVE && ConfigVariables.General.DEBUG_MODE) {
            RobotStateStore.save(drive.localizer.getPose(), lowSlide.getCurrentPosition(),
                    upSlide.getCurrentPosition());
        }
        cleanup();
    }

    private void cleanup() {
        scheduler.cancelAll();
        upSlide.stop();
        lowSlide.stop();
        scheduler.reset();
    }

    private void initializeSubsystems() {
        if (ConfigVariables.General.WITH_STATESAVE) {
            drive = new MecanumDrive(hardwareMap, RobotStateStore.getPose());
        } else {
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        }

        // Initialize BulkReadManager for performance optimization
        bulkReadManager = new BulkReadManager(hardwareMap);

        upSlide = new UpperSlide();
        lowSlide = new LowerSlide();
        camera = new Limelight();
        colorSensor = new ColorSensorImpl(hardwareMap);
        hanging = new Hanging();
        scheduler.registerSubsystem(upSlide);
        scheduler.registerSubsystem(lowSlide);

        upSlide.initialize(hardwareMap);
        lowSlide.initialize(hardwareMap);
        hanging.initialize(hardwareMap);
        camera.initialize(hardwareMap);
        camera.cameraStart();

        // dashboard has LL streaming now
        // only enable limelight forward if in debug mode
        // if(ConfigVariables.General.DEBUG_MODE){
        // LimeLightImageTools llIt = new LimeLightImageTools(camera.limelight);
        // llIt.setDriverStationStreamSource();
        // llIt.forwardAll();
        // }

        upslideActions = new UpperSlideCommands(upSlide);
        lowslideActions = new LowerSlideCommands(lowSlide);

        upperClaw = new ClawController(upSlide::openClaw, upSlide::closeClaw);
        upperExtendo = new ClawController(upSlide::openExtendoClaw, upSlide::closeExtendoClaw);

        lowerClaw = new ClawController(lowSlide::openClaw, lowSlide::closeClaw);

        gamepad1Controller = new GamepadController(gamepad1);
        gamepad2Controller = new GamepadController(gamepad2);

        scheduler.schedule(new ActionCommand(upslideActions.front()));
        scheduler.schedule(new ActionCommand(lowslideActions.up()));
    }

    private void setupGamepadControlsSingle() {
        gamepad1Controller.onPressed(ButtonType.LEFT_BUMPER, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.front()));
        });
        gamepad1Controller.onPressed(ButtonType.RIGHT_BUMPER, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.scorespec()));
        });

        gamepad1Controller.onPressed(ButtonType.A, () -> {
            // Use motion profiling for smooth lower slide retraction
            scheduler.schedule(MotionProfiledSlideCommand.lowerSlideOnly(lowSlide, 0.0));
        });

        gamepad1Controller.onPressed(ButtonType.B, () -> {
            // Use motion profiling for smooth upper slide retraction
            scheduler.schedule(MotionProfiledSlideCommand.upperSlideOnly(upSlide, 0.0));
        });
        gamepad1Controller.onPressed(ButtonType.X, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.hover()));
            scheduler.schedule(new WaitForConditionCommand(
                    () -> gamepad1Controller.getNoOperationTime() > ConfigVariables.Camera.CAMERA_DELAY * 1000, 2500,
                    new AngleAdjustCommand(lowSlide, camera)));
        });
        gamepad1Controller.onPressed(ButtonType.Y, () -> {
            scheduler.schedule(new WaitForConditionCommand(
                    () -> gamepad1Controller.getNoOperationTime() > ConfigVariables.Camera.CAMERA_DELAY * 1000, 3000,
                    new SequentialCommandGroup(new CameraUpdateDetectorResult(camera),
                            new DistanceAdjustLUTThetaR(lowSlide, drive, camera::getTx, camera::getTy, camera::getPx,
                                    camera::getPy, mecanumDriveCommand::disableControl,
                                    mecanumDriveCommand::enableControl))));
        });
        gamepad1Controller.onPressed(ButtonType.DPAD_DOWN, () -> { // drop, for testing
            scheduler.schedule(new SequentialCommandGroup(new ActionCommand(upslideActions.front()),
                    new WaitCommand(ConfigVariables.UpperSlideVars.FRONT_DELAY),
                    new ActionCommand(upslideActions.openClaw())));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_LEFT, () -> {
            scheduler.schedule(new UpperSlideScoreCommand(upslideActions));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_RIGHT, () -> {
            scheduler.schedule(new LowerUpperTransferSequenceCommand(lowslideActions, upslideActions,
                    colorSensor::caughtDefaultTrue));
        });
        gamepad1Controller.onPressed(ButtonType.LEFT_STICK_BUTTON, () -> {
            scheduler.schedule(new SequentialCommandGroup(new ActionCommand((packet) -> {
                mecanumDriveCommand.disableControl();
                return false;
            }), new ActionCommand(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(SCORE.pos, SCORE.heading).build()), new ActionCommand((packet) -> {
                        mecanumDriveCommand.enableControl();
                        return false;
                    })));
        });
    }

    private void setupGamepadControls() {
        gamepad1Controller.onPressed(ButtonType.X, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.hover()));
            scheduler.schedule(new WaitForConditionCommand(
                    () -> gamepad1Controller.getNoOperationTime() > ConfigVariables.Camera.CAMERA_DELAY * 1000, 2500,
                    new AngleAdjustCommand(lowSlide, camera)));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_UP, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.hover()));
            scheduler.schedule(new AngleAdjustCommand(lowSlide, camera));
        });

        gamepad1Controller.onPressed(ButtonType.RIGHT_BUMPER, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.hover()));
        });

        gamepad1Controller.onPressed(ButtonType.A, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.slidePos0()));
        });

        gamepad1Controller.onPressed(ButtonType.B, () -> {
            scheduler.schedule(new CameraUpdateDetectorResult(camera));
            scheduler.schedule(new DistanceAdjustLUTX(drive, camera::getTx, camera::getTy, camera::getPx, camera::getPy,
                    mecanumDriveCommand::disableControl, mecanumDriveCommand::enableControl));
            // scheduler.schedule(new DistanceAdjustCalculatedY(lowSlide, camera::getDy));
            // scheduler.schedule(new DistanceAdjustCalculatedX(drive, camera::getDx,
            // camera::getDy,
            // mecanumDriveCommand::disableControl, mecanumDriveCommand::enableControl));
        });

        gamepad1Controller.onPressed(ButtonType.Y, () -> {
            scheduler.schedule(new WaitForConditionCommand(
                    () -> gamepad1Controller.getNoOperationTime() > ConfigVariables.Camera.CAMERA_DELAY * 1000, 3000,
                    new SequentialCommandGroup(new CameraUpdateDetectorResult(camera),
                            new DistanceAdjustLUTThetaR(lowSlide, drive, camera::getTx, camera::getTy, camera::getPx,
                                    camera::getPy, mecanumDriveCommand::disableControl,
                                    mecanumDriveCommand::enableControl))));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_DOWN, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.setSpinClawDeg(ConfigVariables.Camera.CLAW_90 - 45)));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_LEFT, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.setSpinClawDeg(ConfigVariables.Camera.CLAW_90 - 90)));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_RIGHT, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.setSpinClawDeg(ConfigVariables.Camera.CLAW_90 + 45)));
        });

        gamepad1Controller.onPressed(gamepad1Controller.button(ButtonType.LEFT_BUMPER),
                () -> lowerClaw.handleManualControl(System.currentTimeMillis()));

        gamepad2Controller.onPressed(ButtonType.A, () -> {
            // Use motion profiling for smooth upper slide retraction
            scheduler.schedule(MotionProfiledSlideCommand.upperSlideOnly(upSlide, 0.0));
        });
        // adding tick not pos, direction reversed
        gamepad2Controller.whilePressed(ButtonType.X, (d) -> {
            scheduler.schedule(new ActionCommand(upslideActions.addSlideTick(1)));
        });

        gamepad2Controller.whilePressed(ButtonType.Y, (d) -> {
            scheduler.schedule(new ActionCommand(upslideActions.addSlideTick(-1)));
        });

        gamepad2Controller.onPressed(ButtonType.B, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.slidePos3()));
        });

        gamepad2Controller.onPressed(ButtonType.DPAD_DOWN, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.inter()));
        });

        gamepad2Controller.onPressed(ButtonType.DPAD_UP, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.front()));
        });

        gamepad2Controller.onPressed(ButtonType.DPAD_LEFT, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.offwall()));
        });

        gamepad2Controller.onPressed(ButtonType.DPAD_RIGHT, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.transfer()));
        });

        gamepad2Controller.onPressed(ButtonType.RIGHT_STICK_BUTTON, () -> {
            scheduler.schedule(new LowerUpperTransferSequenceCommand(lowslideActions, upslideActions,
                    colorSensor::caughtDefaultTrue));
        });
        gamepad2Controller.onPressed(ButtonType.LEFT_STICK_BUTTON, () -> {
            scheduler.schedule(new SequentialCommandGroup(new ActionCommand((packet) -> {
                mecanumDriveCommand.disableControl();
                return false;
            }), new ActionCommand(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(SCORE.pos, SCORE.heading).build()), new ActionCommand((packet) -> {
                        mecanumDriveCommand.enableControl();
                        return false;
                    })));
        });
        gamepad2Controller.onPressed(gamepad2Controller.button(ButtonType.LEFT_BUMPER),
                () -> upperClaw.handleManualControl(System.currentTimeMillis()));

        gamepad2Controller.onPressed(gamepad2Controller.button(ButtonType.RIGHT_BUMPER),
                () -> upperExtendo.handleManualControl(System.currentTimeMillis()));
    }

    private void setContinuousControls() {
        gamepad1Controller.onPressed(gamepad1Controller.trigger(GamepadController.TriggerType.RIGHT_TRIGGER), () -> {
            scheduler.schedule(new SequentialCommandGroup(new LowerSlideGrabSequenceCommand(lowSlide),
                    new WaitCommand((double) ConfigVariables.LowerSlideVars.POS_HOVER_TIMEOUT / 1000),
                    new ConditionalCommand(colorSensor::canTransfer, new LowerUpperTransferSequenceCommand(
                            lowslideActions, upslideActions, colorSensor::caughtDefaultTrue))));
        });
        gamepad1Controller.onPressed(gamepad1Controller.trigger(GamepadController.TriggerType.LEFT_TRIGGER), () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.up()));
        });

        gamepad2Controller.onPressed(gamepad2Controller.trigger(GamepadController.TriggerType.RIGHT_TRIGGER), () -> {
            scheduler.schedule(new ActionCommand(upslideActions.scorespec()));
        });

        gamepad2Controller.onPressed(gamepad2Controller.trigger(GamepadController.TriggerType.LEFT_TRIGGER), () -> {
            scheduler.schedule(new ActionCommand(upslideActions.front()));
        });
        gamepad2Controller.onPressed( () -> gamepad2.right_stick_y > 0.5, () -> {
            scheduler.schedule(new HangingCommand(hanging, HangingCommand.Direction.FORWARD));
        });
        gamepad2Controller.onPressed( () -> gamepad2.right_stick_y < -0.5, () -> {
            scheduler.schedule(new HangingCommand(hanging, HangingCommand.Direction.BACKWARD));
        });
        gamepad2Controller.onPressed( () -> Math.abs(gamepad2.right_stick_y - 0) < 0.5, () -> {
            scheduler.schedule(new HangingCommand(hanging, HangingCommand.Direction.STOP));
        });
        gamepad2Controller.onPressed(gamepad2Controller.trigger(GamepadController.TriggerType.LEFT_TRIGGER), () -> {
            scheduler.schedule(new ActionCommand(upslideActions.front()));
        });
    }

}
