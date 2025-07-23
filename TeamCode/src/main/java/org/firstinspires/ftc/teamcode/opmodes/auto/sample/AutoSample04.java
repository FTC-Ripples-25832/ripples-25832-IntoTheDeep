package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
//import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
//import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.CameraUpdateDetectorResult;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTX;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTY;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

@Autonomous
public final class AutoSample04 extends LinearOpMode {
        private MecanumDrive drive;

        private LowerSlide lowSlide;
        private LowerSlideCommands lowerSlideCommands;
        private UpperSlide upSlide;
        private UpperSlideCommands upperSlideCommands;

        private Limelight camera;

        private Action waitSeconds(Pose2d pose, double seconds) {
                return drive.actionBuilder(pose)
                                .waitSeconds(seconds)
                                .build();
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

        private SequentialAction scoreSequence(RobotPosition startPOS, double lowerslideExtendLength) {
                return new SequentialAction(
                                new ParallelAction(
                                                // Drive to score
                                                drive.actionBuilder(startPOS.pose)
                                                                .strafeToLinearHeading(aSCORE.pos, aSCORE.heading)
                                                                .build(),
                                                upperSlideCommands.closeClaw(),
                                                upperSlideCommands.front(),
                                                upperSlideCommands.slidePos3() // need scorespec or transfer pos to go
                                                                               // up safely
                                ),

                                // front pos for drop

                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.A_DROPDELAY_S),
                                upperSlideCommands.openExtendoClaw(),
                                waitSeconds(aSCORE.pose, ConfigVariables.AutoTesting.A_DROPDELAY_S),

                                new ParallelAction(
                                                new SequentialAction(
                                                                upperSlideCommands.openClaw(), // drop
                                                                // SCORED
                                                                waitSeconds(aSCORE.pose,
                                                                                ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S),
                                                                new ParallelAction(
                                                                                upperSlideCommands.closeExtendoClaw(),
                                                                                upperSlideCommands.scorespec()
                                                                // score spec position for upperslides to go downa
                                                                ),
                                                                upperSlideCommands.slidePos0()),

                                                // lowerslide prepare for next cycle
                                                lowerSlideCommands.hover()));

        }

        private SequentialAction pickupAndScoreSequence(RobotPosition startPOS, AutoPaths.RobotPosition pickupPos,
                        double lowerslideExtendLength) {
                return new SequentialAction(
                                // Drive to pickup

                                upperSlideCommands.slidePos0(),
                                drive.actionBuilder(startPOS.pose)
                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                .build(),

                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.Y_PICKUPDELAY),

                                adjustSequence(),

                                new ParallelAction(
                                                new AngleAdjustCommand(lowSlide, camera).toAction(),
                                                // Grab
                                                pickupSequence()),

                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.C_AFTERGRABDELAY_S),
                                // retract, remember to keep pos_hover() when retracting slides
                                lowerSlideCommands.slidePos0(),
                                // lowerSlideCommands.zero(hardwareMap),

                                // transfer sequence
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.D_SLIDEPOS0AFTERDELAY_S),
                                transferSequence(),
                                // Score
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.H_TRANSFERCOMPLETEAFTERDELAY_S),
                                scoreSequence(pickupPos, lowerslideExtendLength));
        }

        @Override
        public void runOpMode() throws InterruptedException {
                // Initialize subsystems
                lowSlide = new LowerSlide();
                upSlide = new UpperSlide();
                lowSlide.initialize(hardwareMap);
                upSlide.initialize(hardwareMap);

                // Initialize command factories
                lowerSlideCommands = new LowerSlideCommands(lowSlide);
                upperSlideCommands = new UpperSlideCommands(upSlide);

                // cam
                camera = new Limelight();
                camera.initialize(hardwareMap);

                // Initialize drive with starting pose
                drive = new MecanumDrive(hardwareMap, START.pose);

                // Start position
                Actions.runBlocking(
                                new SequentialAction(
                                                lowerSlideCommands.up(),
                                                upperSlideCommands.scorespec(),
                                                upperSlideCommands.closeClaw()));

                waitForStart();
                camera.cameraStart();
                if (isStopRequested())
                        return;

                // Full autonomous sequence
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                new SequentialAction(
                                                                upperSlideCommands.scorespec(),

                                                                scoreSequence(aSTART,
                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST),

                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(aSCORE, aPICKUP1,
                                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO)),
                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(aSCORE, aPICKUP2,
                                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO)),

                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(aSCORE, aPICKUP3,
                                                                                                0),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO
                                                                                                                + 90)),

                                                                // end pos for teleop
                                                                upperSlideCommands.setSlidePos(0),
                                                                lowerSlideCommands.up(),
                                                                upperSlideCommands.front(),
                                                                lowerSlideCommands.slidePos0(),
                                                                upperSlideCommands.slidePos0()
                                                // ,drive.actionBuilder(SCORE.pose)
                                                // .turnTo(TELEOP_START.heading)
                                                // .build(),
                                                //
                                                // drive.actionBuilder(new Pose2d(
                                                // SCORE.pos,
                                                // TELEOP_START.heading))
                                                // .strafeToLinearHeading(TELEOP_START.pos,
                                                // TELEOP_START.heading)
                                                // .build()
                                                )));
        }
}
