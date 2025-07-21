package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.PICKUP1;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.PICKUP2;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.PICKUP3;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.RobotPosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.SCORE;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.START;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.base.SaveRobotStateCommand;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.LowerUpperTransferSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.MotionProfiledSlideCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.CameraUpdateDetectorResult;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTThetaR;
import org.firstinspires.ftc.teamcode.opmodes.auto.SubmersibleSelectionGUI;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths;
// import org.firstinspires.ftc.teamcode.opmodes.auto.AutoSelection;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
// import org.firstinspires.ftc.teamcode.sensors.limelight.LimeLightImageTools;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.RobotStateStore;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@Autonomous(name = "A. Sample Cycle Auto", preselectTeleOp = "A. Teleop")
public final class AutoSample extends LinearOpMode {
        private MecanumDrive drive;

        private LowerSlide lowSlide;
        private LowerSlideCommands lowerSlideCommands;
        private UpperSlide upSlide;
        private UpperSlideCommands upperSlideCommands;
        private BulkReadManager bulkReadManager;

        private Limelight camera;

        private Action waitSeconds(Pose2d pose, double seconds) {
                return drive.actionBuilder(pose)
                                .waitSeconds(seconds)
                                .build();
        }

        // --- Helper for repeated drop/reset sequence ---
        private SequentialAction dropAndResetUpperSlides() {
                return new SequentialAction(
                                upperSlideCommands.openClaw(), // drop
                                // SCORED
                                new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S).toAction(),
                                new ParallelAction(
                                                // upperSlideCommands.closeExtendoClaw(),
                                                upperSlideCommands.scorespec()
                                // score spec position for upperslides to go down
                                ),
                                // Use motion profiling for smooth slide retraction
                                upperSlideCommands.slidePos0());
        }

        // --- Helper for repeated front-for-drop sequence ---
        private SequentialAction frontForDrop() {
                return new SequentialAction(
                                upperSlideCommands.front(),
                                new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction());
        }


        private ParallelAction prepUpperSlides() {
                return new ParallelAction(
                        upperSlideCommands.closeClaw(),
                        upperSlideCommands.front(),
                        upperSlideCommands.slidePos3() // need scorespec or transfer pos to go up safely
                );
        }
        private Action driveToScore(RobotPosition startPOS, RobotPosition scorePOS) {
                return drive.actionBuilder(startPOS.pose)
                                .strafeToSplineHeading(scorePOS.pos, scorePOS.heading)
                                .build();
        }


        // --- Helper for repeated drive-to-score and prep upper slides ---
        private ParallelAction driveToScoreAndPrepUpperSlides(RobotPosition startPOS, RobotPosition scorePOS) {
                return new ParallelAction(
                                // Drive to score
                                driveToScore(startPOS, scorePOS),
                                prepUpperSlides()
                );
        }

        // --- Helper for repeated transfer-while-driving sequence ---
        private SequentialAction transferWhileDriving() {
                return new SequentialAction(
                                transferSequence(),
                                // upperSlideCommands.closeClaw(),
                                new WaitCommand(ConfigVariables.AutoTesting.X_TRANSFERWHILEDRIVEAFTERTRANSFERDELAY_S)
                                                .toAction(),
                                upperSlideCommands.inter(),
                                upperSlideCommands.slidePos3() // need scorespec or inter or transfer pos to go up
                // safely
                );
        }

        private SequentialAction scoreSequence(RobotPosition startPOS, RobotPosition scorePOS) {
                return new SequentialAction(
                                driveToScoreAndPrepUpperSlides(startPOS, scorePOS),
                                // front pos for drop
                                // new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction(),
                                // upperSlideCommands.openExtendoClaw(),
                                // new WaitCommand(ConfigVariables.AutoTesting.W_AFTEREXTENDOOPEN_S).toAction(),
                                dropAndResetUpperSlides());
        }

        private SequentialAction transferAndScoreSequence(RobotPosition startPOS, RobotPosition scorePOS) {
                return new SequentialAction(
                                new ParallelAction(
                                                driveToScore(startPOS, scorePOS),
                                                transferWhileDriving()),
                                // front pos for drop
                                // upperSlideCommands.openExtendoClaw(),
                                // new WaitCommand(ConfigVariables.AutoTesting.W_AFTEREXTENDOOPEN_S).toAction(),
                                frontForDrop(),
                                dropAndResetUpperSlides());
        }

        private SequentialAction pickupAndScoreSequence(RobotPosition startPOS, RobotPosition pickupPos, RobotPosition scorePOS,
                        double lowerslideExtendLength) {
                return new SequentialAction(
                                // Drive to pickup
                                // new SetDriveSpeedCommand(40).toAction(),

                                new ParallelAction(
                                                driveToScore(
                                                        startPOS, pickupPos
                                                ),
                                                // Use motion profiling for smooth lower slide extension
                                                lowerSlideCommands.setSlidePos(lowerslideExtendLength)), //pre aim

                                new WaitCommand(ConfigVariables.AutoTesting.Y_PICKUPDELAY).toAction(),
                                adjustAndPickupSequence(),
                                // waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.C_AFTERGRABDELAY_S),

                                // transferSequence(pickupPos),

                                // Score
                                // waitSeconds(pickupPos.pose,
                                // ConfigVariables.AutoTesting.H_TRANSFERCOMPLETEAFTERDELAY_S),
                                transferAndScoreSequence(pickupPos, scorePOS));
        }

        private SequentialAction adjustAndPickupSequence() {
                return new SequentialAction(
                        adjustSequence(),
                        pickupSequence()
                        // waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.C_AFTERGRABDELAY_S),
                );
        }


        private Action transferSequence() {
                return new LowerUpperTransferSequenceCommand(lowerSlideCommands, upperSlideCommands).toAction();
        }

        private Action pickupSequence() {
                return new LowerSlideGrabSequenceCommand(lowSlide).toAction();
        }

        private Action adjustSequence() {
                return new SequentialAction(
                                new CameraUpdateDetectorResult(camera).toAction(),
                                new DistanceAdjustLUTThetaR(lowSlide, drive,
                                                camera::getTx, camera::getTy, camera::getPx, camera::getPy,
                                                () -> {
                                                }, () -> {
                                                }).toAction());
        }

        @Override
        public void runOpMode() throws InterruptedException {
                // Initialize BulkReadManager for performance optimization
                bulkReadManager = new BulkReadManager(hardwareMap);

                // Initialize subsystems
                lowSlide = new LowerSlide();
                upSlide = new UpperSlide();
                lowSlide.initialize(hardwareMap);
                upSlide.initialize(hardwareMap);

                // Initialize command factories
                lowerSlideCommands = new LowerSlideCommands(lowSlide);
                upperSlideCommands = new UpperSlideCommands(upSlide);

                camera = new Limelight();
                camera.initialize(hardwareMap);
                camera.cameraStart();

                // DISABLED FOR PERFORMANCE: Limelight processing adds ms per loop
                // LimeLightImageTools llIt = new LimeLightImageTools(camera.limelight);
                // llIt.setDriverStationStreamSource();
                // llIt.forwardAll();
                // FtcDashboard.getInstance().startCameraStream(llIt.getStreamSource(), 10);

                // Initialize drive with starting pose
                drive = new MecanumDrive(hardwareMap, START.pose);

                // Start position
                Actions.runBlocking(
                                new SequentialAction(
                                                lowerSlideCommands.up(),
                                                upperSlideCommands.scorespec(),
                                                upperSlideCommands.closeClaw()));

                String color = "blue"; // Default color
                ElapsedTime elapsedTime;
                elapsedTime = new ElapsedTime();
                boolean colorSelected = false;

                // SubmersibleSelectionGUI integration
                SubmersibleSelectionGUI gui = new SubmersibleSelectionGUI();

                while (!isStarted() && !isStopRequested()) {
                        // Color selection
                        // AutoSelection autoSelection = new AutoSelection(gamepad1); // broken, dont
                        // need
                        // js do this here
                        while (!colorSelected) {
                                telemetry.addLine(
                                        "Use D-Pad Right to change color for sample detection, A to go to next phase");
                                telemetry.addData("selected color: ", color);
                                telemetry.update();

                                if (0.25 < elapsedTime.seconds() && gamepad1.dpad_right) {
                                        elapsedTime.reset();
                                        color = color == "red" ? "blue" : "red";
                                }

                                if(gamepad1.a){
                                        colorSelected = true;
                                }
                        }

                        // autoSelection.updateTelemetry(telemetry); unused
                        telemetry.addLine(
                                "Use D-Pad/joystick to select pickup points, A/LB to toggle, Y to preset. Press start when done.");
                        //rm this addline if redundant
                        telemetry.update();
                        // gui selection
                        gui.drawSub(gamepad1, telemetry);
                        sleep(50);
                }

                // Set Limelight color based on selection
                // String selectedColor = autoSelection.getColor().toString().toLowerCase(); //
                // "red" or "blue"
                // String selectedColor = color;
                if (color.equals("blue")) { // cuz this sample, need yellow
                        camera.setAcceptedColors(true, false, true); // blue, not red, yellow
                } else if (color.equals("red")) {
                        camera.setAcceptedColors(false, true, true); // not blue, red, yellow
                } else {
                        camera.setAcceptedColors(false, false, true); // fallback: yellow
                }

                ArrayList<Pose2d> selectedPickupPoints = gui.getDriverSelect();
                // If none selected, use fallback
                boolean useFallback = selectedPickupPoints.isEmpty();
                // Define pickupPoints as a list of Vector2d
                List<Vector2d> pickupPoints;
                if (useFallback) {
                        // Fallback to current hardcoded path
                        pickupPoints = List.of(
                                        new Vector2d(39, 28),
                                        new Vector2d(39, 0));
                } else {
                        // Use selected points for pickup cycles
                        pickupPoints = selectedPickupPoints.stream()
                                        .map(p -> new Vector2d(p.position.x + 20, p.position.y)) // PLUS 20 OFFSET SO SLILDES CAN MAX
                                        .collect(Collectors.toList());
                }

                waitForStart();

                if (isStopRequested())
                        return;

                // Full autonomous sequence
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),

                                                // PERFORMANCE OPTIMIZATION: Add bulk read manager to action system
                                                packet -> {
                                                        bulkReadManager.updateBulkRead();
                                                        return true; // Always continue running
                                                },

                                                // Add camera telemetry for debugging
                                                packet -> {
                                                        camera.updateTelemetry(packet);
                                                        return true; // Always continue running
                                                },
                                                new SequentialAction(
                                                                upperSlideCommands.scorespec(),
                                                                new ParallelAction(
                                                                        driveToScore(START, PICKUP1), // inited sample
                                                                        prepUpperSlides(),
                                                                        lowerSlideCommands.setSlidePos(ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST), // pre aim
                                                                        lowerSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO)
                                                                ),
                                                                new ParallelAction( // start pickup when scoring init sample
                                                                                dropAndResetUpperSlides(),
                                                                                new SequentialAction(
                                                                                        adjustAndPickupSequence(),

                                                                                        new ParallelAction(
                                                                                                driveToScore(PICKUP1, PICKUP2),
                                                                                                new SequentialAction(
                                                                                                        transferSequence(),
                                                                                                        new WaitCommand(ConfigVariables.AutoTesting.X_TRANSFERWHILEDRIVEAFTERTRANSFERDELAY_S).toAction(),
                                                                                                        upperSlideCommands.inter()
                                                                                                )
//                                                                                                transferWhileDriving()
                                                                                        ),
                                                                                        // transferAndScoreSequence(PICKUP1, PICKUP2)
                                                                                        new ParallelAction(
                                                                                                new SequentialAction( // score first pickuped
                                                                                                        upperSlideCommands.slidePos3(),
                                                                                                        frontForDrop(),
                                                                                                        dropAndResetUpperSlides()
                                                                                                ),
                                                                                                lowerSlideCommands.setSlidePos(ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND),
                                                                                                lowerSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 45),
                                                                                                new SequentialAction(
                                                                                                        new WaitCommand(ConfigVariables.AutoTesting.Y_PICKUPDELAY).toAction(),
                                                                                                        adjustAndPickupSequence() // start adjust second while scoring
                                                                                                )
                                                                                        ),
                                                                                        transferAndScoreSequence(PICKUP2, SCORE) // score second
                                                                                )
//                                                                                lowerSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO)),
//                                                                                pickupAndScoreSequence(PICKUP1, PICKUP1, PICKUP2, ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST),
                                                                ),
//                                                                new ParallelAction(
//                                                                                pickupAndScoreSequence(PICKUP2, PICKUP2, SCORE, ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND),
//                                                                                lowerSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 45)),
                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(SCORE, PICKUP3, SCORE, ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD),
                                                                                lowerSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 90)),

                                                                // full send

                                                                // original
                                                                // new ParallelAction(
                                                                // drive.actionBuilder(SCORE.pose)
                                                                // .strafeTo(new Vector2d(
                                                                // 39, 28))
                                                                // .splineTo(new Vector2d(
                                                                // 30, 15),
                                                                // Math.toRadians(205))
                                                                // .build(),
                                                                // lowerSlideCommands.slidePos1()
                                                                // ),

                                                                // Convert pickup points to individual actions and run
                                                                // them sequentially
                                                                new SequentialAction(
                                                                                pickupPoints.stream()
                                                                                                .map(pickupVec -> (Action) new SequentialAction(
                                                                                                                new ParallelAction(
                                                                                                                                drive.actionBuilder(
                                                                                                                                                SCORE.pose)
                                                                                                                                                .strafeTo(new Vector2d(
                                                                                                                                                                39,
                                                                                                                                                                28))
                                                                                                                                                .splineTo(pickupVec, Math.toRadians(-160))
                                                                                                                                                .build(),
                                                                                                                                lowerSlideCommands
                                                                                                                                                .slidePos1()),

                                                                                                                new WaitCommand(ConfigVariables.AutoTesting.I_SUBDELAY_S)
                                                                                                                                .toAction(),

                                                                                                                adjustSequence(),
                                                                                                                new WaitCommand(ConfigVariables.Camera.CAMERA_DELAY)
                                                                                                                                .toAction(),
                                                                                                                new AngleAdjustCommand(
                                                                                                                                lowSlide,
                                                                                                                                camera)
                                                                                                                                .toAction(),
                                                                                                                pickupSequence(),

                                                                                                                new ParallelAction(
                                                                                                                                drive.actionBuilder(
                                                                                                                                                drive.localizer.getPose())
                                                                                                                                                .setReversed(true)
                                                                                                                                                .splineTo(new Vector2d(
                                                                                                                                                                44,
                                                                                                                                                                28),
                                                                                                                                                                SCORE.heading - Math
                                                                                                                                                                                .toRadians(170))
                                                                                                                                                .splineTo(SCORE.pos,
                                                                                                                                                                SCORE.heading - Math
                                                                                                                                                                                .toRadians(170))
                                                                                                                                                .build(),
                                                                                                                                new SequentialAction(
                                                                                                                                        new WaitCommand(ConfigVariables.AutoTesting.K_ROUNDPATHEXITTIME_S).toAction(),
                                                                                                                                        transferWhileDriving()
                                                                                                                                )
                                                                                                                                ),
                                                                                                                frontForDrop(),
                                                                                                                dropAndResetUpperSlides()))
                                                                                                .toArray(Action[]::new)))));
                RobotStateStore.save(drive.localizer.getPose(), lowSlide.getCurrentPosition(), upSlide.getCurrentPosition());
        }
}
