package team3647.frc2024.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2024.constants.AutoConstants;
import team3647.frc2024.constants.FieldConstants;
import team3647.frc2024.subsystems.Superstructure;
import team3647.frc2024.subsystems.SwerveDrive;
import team3647.frc2024.util.AllianceFlip;
import team3647.frc2024.util.AutoDrive.DriveMode;
import team3647.frc2024.util.TargetingUtil;
import team3647.lib.team9442.AllianceObserver;

public class AutoCommands implements AllianceObserver {
    private final SwerveDrive swerve;
    private final Supplier<Twist2d> autoDriveVelocities;
    private final Superstructure superstructure;
    private final TargetingUtil targeting;

    private final String s1_to_n1 = "s1 to n1";
    private final String s1_to_n1_to_f1 = "s1 to n1 to f1";
    private final String s2_to_n2a = "s2 to n2";
    private final String n2_to_n1 = "n2 to n1";
    private final String n2_to_n3 = "n2 to n3";
    private final String s3_to_n3 = "s3 to n3";
    private final String n3_to_f5 = "n3 to f5";
    private final String n1_to_n2 = "n1 to n2";
    private final String n3_to_n2 = "n3 to n2";
    private final String shoot1_to_n2 = "shoot1 to n2";
    private final String shoot1_to_n1 = "shoot1 to n1";
    private final String s15_to_f1 = "s15 to f1";
    private final String f1_to_n2 = "f1 to n2";
    private final String f1_to_shoot1 = "f1 to shoot1";
    private final String shoot1_to_f2 = "shoot1 to f2";
    private final String f2_to_n2 = "f2 ton2";
    private final String n2_to_f1 = "n2 to f1";
    private final String f3_to_shoot1 = "f3 to shoot1";
    private final String f2_to_shoot1 = "f2 to shoot1";
    private final String shoot1_to_f3 = "shoot1 to f3";
    private final String f3_to_shoot2 = "f3 to shoot2";
    private final String f5_to_shoot3 = "f5 to shoot3";
    private final String shoot3_to_f4 = "shoot3 to f4";
    private final String f4_to_shoot3 = "f4 to shoot3";
    private final String shoot3_to_f3 = "shoot3 to f3";
    private final String n1_to_f1 = "n1 to f1";
    private final String s3_to_f5 = "s3 to f5";
    private final String f2_to_n1 = "f2 to n1";
    private final String s35_to_f5 = "s35 to f5";
    private final String s2_to_f3 = "s2 to f3";
    private final String shoot2_to_f4 = "shoot2 to f4";
    private final String f1_to_f2 = "f1 to f2";
    private final String f2_to_f3 = "f2 to f3";
    private final String f3_to_f4 = "f3 to f4";
    private final String f4_to_f5 = "f4 to f5";
    private final String shoot3_to_f5 = "shoot3 to f5";
    private final String shoot4_to_f4 = "shoot4 to f4";
    private final String f3_to_shoot4 = "f3 to shoot4";
    private final String shoot1_to_f1 = "shoot1 to f1";
    private final String shoot2_to_f2 = "shoot2 to f2";
    private final String f3_to_shoot3 = "f3 to shoot3";
    private final String shoot3_to_f2 = "shoot3 to f2";
    private final String f2_to_shoot3 = "f2 to shoot3";
    private final String shoot3_to_f1 = "shoot3 to f1";
    private final String f1_to_shoot3 = "shoot3 to f1";
    private final String shoot2_to_f5 = "shoot2 to f5";
    private final String f4_to_shoot2 = "f4 to shoot2";

    private final String s15_straight_forward = "s15 straight forward";
    private final String trap_test = "trap test 2";
    private final String s3_preload_move = "s3 preload and move";

    public final Trigger currentYes;

    private boolean hasPiece = true;

    private final BooleanSupplier hasTarget;

    double SLOWDOWN = 1;

    public final BooleanSupplier noteNotSeen;

    private final Supplier<DriveMode> getMode;

    private final DoubleSupplier vel90;

    private final DoubleSupplier velOther90;

    private final DoubleSupplier driveX;

    private final DoubleSupplier onTheMoveRotSupplier;

    //     public final AutonomousMode blueFive_S1N1F1N2N3;

    public final AutonomousMode blueFour_S1N1N2N3;

    public final AutonomousMode blueFour_S1F1F2F3;

    public final AutonomousMode blueFour_S3F5F4F3;

    public final AutonomousMode blueSix_S1F1F2N1N2N3;

    public final AutonomousMode blueForwardTest;

    public final AutonomousMode blueRightTest;

    //     public final AutonomousMode redFive_S1N1F1N2N3;

    public final AutonomousMode redFour_S1F1F2F3;

    public final AutonomousMode redFour_S1N1N2N3;

    public final AutonomousMode redTwo_S2F3;

    public final AutonomousMode redFour_S3F5F4F3;

    //     public final AutonomousMode redFive_S1N1F1F2F3;

    public final AutonomousMode redSix_S1F1F2N1N2N3;

    public final AutonomousMode blueTest_S15;

    public final AutonomousMode blueFullCenterS1;

    public final AutonomousMode redFullCenterS1;

    public final AutonomousMode blueFullCenterS3;

    public final AutonomousMode redFullCenterS3;

    public final AutonomousMode redTest;

    public final AutonomousMode doNothing;

    public final AutonomousMode redPreloadOnly;

    public final AutonomousMode bluePreloadOnly;

    public final AutonomousMode preloadMove;

    public List<AutonomousMode> redAutoModes;

    public List<AutonomousMode> blueAutoModes;

    private MidlineNote lastNote = MidlineNote.ONE;

    Alliance color = Alliance.Red;

    private final PIDController fastXController = new PIDController(2, 0, 0);

    //     public final AutonomousMode yes;

    public AutoCommands(
            SwerveDrive swerve,
            Supplier<Twist2d> autoDriveVelocities,
            DoubleSupplier onTheMoveRotSupplier,
            DoubleSupplier vel90,
            DoubleSupplier velOther90,
            DoubleSupplier xSupplier,
            Superstructure superstructure,
            TargetingUtil targeting,
            BooleanSupplier hasTarget,
            Supplier<DriveMode> getMode) {
        this.onTheMoveRotSupplier = onTheMoveRotSupplier;
        this.swerve = swerve;
        this.autoDriveVelocities = autoDriveVelocities;
        this.superstructure = superstructure;
        this.targeting = targeting;
        this.hasTarget = hasTarget;
        this.getMode = getMode;
        this.vel90 = vel90;
        this.velOther90 = velOther90;
        this.driveX = xSupplier;

        noteNotSeen = new Trigger(() -> !hasTarget.getAsBoolean()).debounce(0.5);

        currentYes = new Trigger(() -> superstructure.currentYes()).debounce(0.06);

        // this.yes = new AutonomousMode(four_S3N5N4N3(Alliance.Blue), getInitial(s3_to_f5));

        // this.blueFive_S1N1F1N2N3 = // DONT USE
        //         new AutonomousMode(five_S1N1F1N2N3(Alliance.Blue), getInitial(s1_to_n1_to_f1));

        // this.redFive_S1N1F1N2N3 = // DONT USE
        //         new AutonomousMode(
        //                 five_S1N1F1N2N3(Alliance.Red),
        //                 AllianceFlip.flipForPP(getInitial(s1_to_n1_to_f1)));

        this.blueTest_S15 =
                new AutonomousMode(
                        testS1(Alliance.Blue), getInitial(s15_straight_forward), "blue test");

        this.redTwo_S2F3 =
                new AutonomousMode(
                        two_S2F3(Alliance.Red),
                        AllianceFlip.flipForPP(getInitial(s2_to_f3)),
                        "red two note mid");
        this.redFour_S3F5F4F3 =
                new AutonomousMode(
                        four_S3F5F4F3(Alliance.Red),
                        AllianceFlip.flipForPP(getInitial(s35_to_f5)),
                        "red 4 source");

        this.redFour_S1N1N2N3 =
                new AutonomousMode(
                        four_S1N1N2N3(Alliance.Red),
                        AllianceFlip.flipForPP(getInitial(s1_to_n1)),
                        "red 4 amp near");

        // this.redFive_S1N1F1F2F3 = // DONT USE
        //         new AutonomousMode(
        //                 five_S1N1F1N2N3(Alliance.Red),
        //                 AllianceFlip.flipForPP(getInitial(s1_to_n1_to_f1)));

        this.redFour_S1F1F2F3 =
                new AutonomousMode(
                        four_S1F1F2F3(Alliance.Red),
                        AllianceFlip.flipForPP(getInitial(s15_to_f1)),
                        "red 4 amp far");

        this.redSix_S1F1F2N1N2N3 =
                new AutonomousMode(
                        six_S1F1F2N1N2N3(Alliance.Red),
                        AllianceFlip.flipForPP(getInitial(s15_to_f1)),
                        "red six note");

        this.blueFour_S1N1N2N3 =
                new AutonomousMode(
                        four_S1N1N2N3(Alliance.Blue), getInitial(s1_to_n1), "blue 4 amp near");

        this.blueSix_S1F1F2N1N2N3 =
                new AutonomousMode(
                        six_S1F1F2N1N2N3(Alliance.Blue), getInitial(s15_to_f1), "blue six note");

        this.blueFour_S3F5F4F3 =
                new AutonomousMode(
                        four_S3F5F4F3(Alliance.Blue), getInitial(s35_to_f5), "blue 4 source far");

        this.blueFour_S1F1F2F3 =
                new AutonomousMode(
                        four_S1F1F2F3(Alliance.Blue), getInitial(s15_to_f1), "blue 4 amp far");

        this.blueFullCenterS1 =
                new AutonomousMode(
                        fullCenterS1(Alliance.Blue), getInitial(s15_to_f1), "blue all mid amp");

        this.redFullCenterS1 =
                new AutonomousMode(
                        fullCenterS1(Alliance.Red),
                        AllianceFlip.flipForPP(getInitial(s15_to_f1)),
                        "red all mid amp");

        this.blueFullCenterS3 =
                new AutonomousMode(
                        fullCenterS3(Alliance.Blue), getInitial(s35_to_f5), "blue all mid source");

        this.redFullCenterS3 =
                new AutonomousMode(
                        fullCenterS3(Alliance.Red),
                        AllianceFlip.flipForPP(getInitial(s35_to_f5)),
                        "red all mid source");

        this.blueForwardTest =
                new AutonomousMode(testForward(), getInitial("test Forward"), "blue forward test");
        this.blueRightTest =
                new AutonomousMode(testRight(), getInitial("test right"), "blue right test");
        this.redTest =
                new AutonomousMode(
                        testForward(), AllianceFlip.flipForPP(getInitial(s15_straight_forward)));
        this.doNothing =
                new AutonomousMode(
                        Commands.none(),
                        AllianceFlip.flipForPP(
                                getInitial(s15_straight_forward), color == Alliance.Red));
        this.redPreloadOnly =
                new AutonomousMode(
                        preloadOnly(),
                        AllianceFlip.flipForPP(getInitial(s1_to_n1)),
                        "red preload only");
        this.bluePreloadOnly =
                new AutonomousMode(preloadOnly(), getInitial(s1_to_n1), "blue preload only");
        this.preloadMove =
                new AutonomousMode(
                        preloadMove(),
                        AllianceFlip.flipForPP(getInitial(s3_preload_move), color == Alliance.Red));

        redAutoModes =
                new ArrayList<AutonomousMode>(
                        Set.of(
                                redPreloadOnly,
                                redFullCenterS1,
                                redFullCenterS3,
                                redFour_S1F1F2F3,
                                redFour_S1N1N2N3,
                                redFour_S3F5F4F3,
                                redSix_S1F1F2N1N2N3,
                                redTwo_S2F3,
                                doNothing));

        blueAutoModes =
                new ArrayList<AutonomousMode>(
                        Set.of(
                                bluePreloadOnly,
                                blueFour_S1F1F2F3,
                                blueFour_S1N1N2N3,
                                blueFour_S3F5F4F3,
                                blueFullCenterS1,
                                blueFullCenterS3,
                                blueSix_S1F1F2N1N2N3,
                                blueForwardTest,
                                blueRightTest,
                                doNothing));
    }

    public enum MidlineNote {
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE
    }

    @Override
    public void onAllianceFound(Alliance color) {
        this.color = color;
    }

    public void registerCommands() {}

    public AutonomousMode getSix_S1F1F2N1N2N3ByColor(Alliance color) {
        return new AutonomousMode(six_S1F1F2N1N2N3(color), getInitial(s1_to_n1_to_f1));
    }

    public AutonomousMode getFive_S1N1F1N2N3ByColor(Alliance color) {
        return new AutonomousMode(five_S1N1F1N2N3(color), getInitial(s1_to_n1_to_f1));
    }

    public AutonomousMode getFour_S1N1F1N2N3ByColor(Alliance color) {
        return new AutonomousMode(four_S1N1N2N3(color), getInitial(s1_to_n1));
    }

    public Command seven_S1N1F1F2F3N2N3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverrideFast(s1_to_n1, color),
                        Commands.waitSeconds(0.2),
                        followChoreoPathWithOverrideFast(n1_to_f1, color),
                        followChoreoPathWithOverrideFast(f1_to_shoot1, color),
                        Commands.waitSeconds(0.2),
                        followChoreoPathWithOverrideFast(shoot1_to_f2, color),
                        followChoreoPathWithOverrideFast(f2_to_shoot1, color),
                        Commands.waitSeconds(0.2),
                        followChoreoPathWithOverrideFast(shoot1_to_f3, color),
                        followChoreoPathWithOverrideFast(f3_to_shoot1, color),
                        Commands.waitSeconds(0.2),
                        followChoreoPathWithOverrideFast(shoot1_to_n2, color),
                        followChoreoPathWithOverrideFast(n2_to_n3, color)));
    }

    public Command getScoringSequenceF1F2(Alliance color) {
        return Commands.sequence(
                followChoreoPathWithOverride(f1_to_shoot1, color),
                target().withTimeout(0.5),
                followChoreoPathWithOverrideFast(shoot1_to_f2, color));
    }

    public Command getScoringSequenceF2F3(Alliance color) {
        return Commands.sequence(
                followChoreoPathWithOverride(f2_to_shoot1, color),
                target().withTimeout(0.8),
                followChoreoPathWithOverrideFast(shoot1_to_f3, color));
    }

    public Command getScoringSequenceF3F4(Alliance color) {
        return Commands.sequence(
                followChoreoPathWithOverride(f3_to_shoot2, color),
                target().withTimeout(0.8),
                followChoreoPathWithOverrideFast(shoot2_to_f4, color));
    }

    public Command getScoringSequenceF4F5(Alliance color) {
        return Commands.sequence(
                followChoreoPathWithOverrideLongTimer(f4_to_shoot2, color),
                target().withTimeout(0.8),
                followChoreoPathWithOverrideFast(shoot2_to_f5, color));
    }

    public Command getScoringSequenceF2F1(Alliance color) {
        return Commands.sequence(
                followChoreoPathWithOverrideLongTimer(f2_to_shoot3, color),
                target().withTimeout(0.8),
                followChoreoPathWithOverrideFast(shoot3_to_f1, color));
    }

    public Command getScoringSequenceF3F2(Alliance color) {
        return Commands.sequence(
                followChoreoPathWithOverride(f3_to_shoot3, color),
                target().withTimeout(0.8),
                followChoreoPathWithOverrideFast(shoot3_to_f2, color));
    }

    public Command getScoringSequenceF4F3(Alliance color) {
        return Commands.sequence(
                followChoreoPathWithOverrideLongTimer(f4_to_shoot3, color),
                target().withTimeout(0.8),
                followChoreoPathWithOverrideFast(shoot3_to_f3, color));
    }

    public Command getScoringSequenceF5F4(Alliance color) {
        return Commands.sequence(
                followChoreoPathWithOverride(f5_to_shoot3, color),
                target().withTimeout(0.8),
                followChoreoPathWithOverrideNoverrideFast(shoot3_to_f4, color));
    }

    public Command testForward() {
        return followChoreoPath("test Forward", Alliance.Blue);
    }

    public Command testRight() {
        return followChoreoPath("test right", Alliance.Blue);
    }

    public Command testS1(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(followChoreoPathWithOverride(s15_straight_forward, color)));
    }

    public Command fullCenterS1(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverrideFast(s15_to_f1, color),
                        searchOrScoreAmpToSource(() -> !noteNotSeen.getAsBoolean(), color)
                                .repeatedly()));
    }

    public Command preloadOnly() {
        return Commands.parallel(target().withTimeout(10), scorePreload());
    }

    public Command preloadMove() {
        return Commands.sequence(
                scorePreload(), Commands.waitSeconds(2), followChoreoPath(s3_preload_move, color));
    }

    public Command fullCenterS3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverrideFast(s35_to_f5, color),
                        searchOrScoreSourceToAmp(() -> !noteNotSeen.getAsBoolean(), color)
                                .repeatedly()));
    }

    public Command six_S1F1F2N1N2N3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverrideFast(s15_to_f1, color),
                        followChoreoPathWithOverride(f1_to_shoot1, color),
                        target().withTimeout(0.5),
                        followChoreoPathWithOverrideFast(shoot1_to_f2, color),
                        followChoreoPathWithOverrideFast(f2_to_n1, color),
                        target().withTimeout(0.2),
                        followChoreoPathWithOverride(n1_to_n2, color),
                        target().withTimeout(0.2),
                        followChoreoPathWithOverride(n2_to_n3, color),
                        target()));
    }

    public Command four_S3N5N4N3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverride(s35_to_f5, color),
                        followChoreoPathWithOverride(f5_to_shoot3, color),
                        followChoreoPathWithOverride(shoot3_to_f3, color),
                        followChoreoPathWithOverride(f3_to_shoot2, color)));
    }

    public Command two_S2F3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverride(s2_to_f3, color),
                        followChoreoPathWithOverrideFast(f3_to_shoot1, color)));
    }

    public Command five_S1N1F1N2N3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverride(s1_to_n1_to_f1, color),
                        followChoreoPathWithOverrideFast(f1_to_n2, color),
                        followChoreoPathWithOverride(n2_to_n3, color)));
    }

    public Command four_S3F5F4F3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverrideFast(s35_to_f5, color),
                        followChoreoPathWithOverride(f5_to_shoot3, color),
                        target().withTimeout(0.4),
                        followChoreoPathWithOverrideFast(shoot4_to_f4, color),
                        followChoreoPathWithOverride(f4_to_shoot3, color),
                        target().withTimeout(0.4),
                        followChoreoPathWithOverrideFast(shoot3_to_f3, color),
                        followChoreoPathWithOverride(f3_to_shoot4, color),
                        target()));
    }

    public Command four_S1F1F2F3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverrideFast(s15_to_f1, color),
                        followChoreoPathWithOverrideFast(f1_to_shoot1, color),
                        followChoreoPathWithOverrideFast(shoot1_to_f2, color),
                        followChoreoPathWithOverrideFast(f2_to_shoot1, color),
                        followChoreoPathWithOverrideFast(shoot1_to_f3, color),
                        followChoreoPathWithOverrideFast(f3_to_shoot1, color)));
    }

    public Command four_S1N1N2N3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(color),
                Commands.sequence(
                        followChoreoPathWithOverride(s1_to_n1, color),
                        target().withTimeout(1),
                        followChoreoPathWithOverride(n1_to_n2, color),
                        target().withTimeout(1),
                        followChoreoPathWithOverride(n2_to_n3, color),
                        target().withTimeout(1)));
    }

    public Command pathToTrapTest() {
        return Commands.sequence(
                pathfindToTrap(),
                Commands.waitSeconds(0.2),
                Commands.parallel(
                        followChoreoPath(trap_test, Alliance.Red),
                        superstructure.trapShot(swerve::getPoseX)));
    }

    public Command pathToAmp(Alliance color) {
        return Commands.sequence(pathfindToAmp(color), followChoreoPath("amp", color));
    }

    public Command pathfindToAmp(Alliance color) {
        return AutoBuilder.pathfindToPose(
                color == Alliance.Blue
                        ? getInitial("amp")
                        : AllianceFlip.flipForPP(getInitial("amp")),
                new PathConstraints(3, 4, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                0,
                0);
    }

    public Command searchAmpToSource() {
        return Commands.run(
                () -> {
                    if (hasTarget.getAsBoolean()) {
                        swerve.drive(autoDriveVelocities.get().dx, autoDriveVelocities.get().dy, 0);
                    } else {
                        swerve.driveFieldOriented(
                                driveX.getAsDouble(),
                                Math.abs(vel90.getAsDouble()) < 1 && targeting.pose().getY() > 0.5
                                        ? -4
                                        : 0,
                                vel90.getAsDouble());
                    }
                },
                swerve);
    }

    public Command searchSourceToAmp() {
        return Commands.run(
                () -> {
                    if (hasTarget.getAsBoolean()) {
                        swerve.drive(autoDriveVelocities.get().dx, autoDriveVelocities.get().dy, 0);
                    } else {
                        swerve.driveFieldOriented(
                                driveX.getAsDouble(),
                                Math.abs(velOther90.getAsDouble()) < 1
                                                && targeting.pose().getY()
                                                        < FieldConstants.kFieldWidth - 0.5
                                        ? 4
                                        : 0,
                                velOther90.getAsDouble());
                    }
                },
                swerve);
    }

    public Command searchOrScoreAmpToSource(BooleanSupplier hasNote, Alliance color) {
        return new ConditionalCommand(
                getScoringSequenceByPoseAmpToSource(color),
                searchAmpToSource()
                        .until(currentYes)
                        .andThen(getScoringSequenceByPoseAmpToSource(color)),
                hasNote);
    }

    public Command searchOrScoreSourceToAmp(BooleanSupplier hasNote, Alliance color) {
        return new ConditionalCommand(
                getScoringSequenceByPoseSourceToAmp(color),
                searchSourceToAmp()
                        .until(currentYes)
                        .andThen(getScoringSequenceByPoseSourceToAmp(color)),
                hasNote);
    }

    public Command getScoringSequenceByPoseAmpToSource(Alliance color) {
        return new SelectCommand<>(
                Map.of(
                        MidlineNote.ONE,
                        getScoringSequenceF1F2(color),
                        MidlineNote.TWO,
                        getScoringSequenceF2F3(color),
                        MidlineNote.THREE,
                        getScoringSequenceF3F4(color),
                        MidlineNote.FOUR,
                        getScoringSequenceF4F5(color),
                        MidlineNote.FIVE,
                        followChoreoPathWithOverride(f1_to_shoot1, color)),
                () -> getNoteNumberByPose(swerve.getOdoPose().getY()));
    }

    public Command getScoringSequenceByPoseSourceToAmp(Alliance color) {
        return new SelectCommand<>(
                Map.of(
                        MidlineNote.ONE,
                        followChoreoPathWithOverride(f1_to_shoot3, color),
                        MidlineNote.TWO,
                        getScoringSequenceF2F1(color),
                        MidlineNote.THREE,
                        getScoringSequenceF3F2(color),
                        MidlineNote.FOUR,
                        getScoringSequenceF4F3(color),
                        MidlineNote.FIVE,
                        getScoringSequenceF5F4(color)),
                () -> getNoteNumberByPose(swerve.getOdoPose().getY()));
    }

    public MidlineNote getNoteNumberByPose(double poseY) {
        if (poseY > 6.6) {
            return MidlineNote.ONE;
        } else if (poseY > 4.95) {
            return MidlineNote.TWO;
        } else if (poseY > 3.3) {
            return MidlineNote.THREE;
        } else if (poseY > 1.6) {
            return MidlineNote.FOUR;
        } else {
            return MidlineNote.FIVE;
        }
    }

    public Command pathfindToTrap() {
        return AutoBuilder.pathfindToPose(
                AllianceFlip.flipForPP(getInitial(trap_test)),
                new PathConstraints(3, 4, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                0,
                0);
    }

    public Command masterSuperstructureSequence(Alliance color) {
        return Commands.sequence(
                poopPreload(),
                Commands.parallel(
                        // superstructure.spinUp(),
                        superstructure.prep(),
                        // superstructure.fastFeed(),
                        continuouslyIntakeForShoot(color).repeatedly(),
                        superstructure.feed()));
    }

    public boolean goodToGo(Alliance color) {
        if (color == Alliance.Blue) {
            return swerve.getOdoPose().getX() < AutoConstants.kDrivetrainXShootingThreshold;
        } else {
            return swerve.getOdoPose().getX()
                    > FieldConstants.kFieldLength - AutoConstants.kDrivetrainXShootingThreshold;
        }
    }

    public Command poopPreload() {
        return Commands.parallel(
                        superstructure.spinUpPreload(),
                        superstructure.prep(),
                        Commands.sequence(Commands.waitSeconds(1), superstructure.feed()))
                .withTimeout(1.4);
    }

    public Command scorePreload() {
        return Commands.parallel(
                        superstructure.spinUp(),
                        superstructure.prep(),
                        Commands.sequence(Commands.waitSeconds(1), superstructure.feed()))
                .withTimeout(1.4);
    }

    public Pose2d getInitial(String path) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        return traj.getInitialPose();
    }

    public Pose2d getFinal(String path) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        return traj.getFinalPose();
    }

    public Command spinUp() {
        return Commands.parallel(superstructure.spinUp(), superstructure.feed());
    }

    //     public Command target() {
    //         return Commands.run(() -> swerve.drive(0, 0, deeThetaOnTheMove()),
    // swerve).withTimeout(2);
    //     }

    public Command shoot() {
        return Commands.parallel(superstructure.shoot());
    }

    public Command continuouslyIntakeForShoot(Alliance color) {
        return Commands.sequence(
                Commands.runOnce(() -> this.hasPiece = false),
                superstructure
                        .intake()
                        .until(currentYes)
                        .andThen(Commands.runOnce(() -> this.hasPiece = true))
                        .andThen(
                                superstructure.passToShooterNoKicker(
                                        new Trigger(() -> goodToGo(color)))));
    }

    public Pose2d flipForPP(Pose2d pose) {
        return new Pose2d(
                new Translation2d(FieldConstants.kFieldLength - pose.getX(), pose.getY()),
                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    }

    public Command overrideChoreoPathWithIntake(String path, Alliance color) {
        return Commands.deadline(pathAndShootWithOverride(path, color), superstructure.intake());
    }

    public Command pathAndShootWithOverride(String path, Alliance color) {
        return Commands.parallel(
                superstructure.shootStow(), followChoreoPathWithOverride(path, color));
    }

    public Command target() {
        return Commands.run(() -> swerve.drive(0, 0, deeTheta()), swerve);
    }

    public Command followChoreoPathWithOverrideFast(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        PathPlannerLogging.logActivePath(PathPlannerPath.fromChoreoTrajectory(path));
        Logger.recordOutput("Autos/current path", path);
        return customChoreoFolloweForOverride(
                        traj,
                        swerve::getOdoPose,
                        choreoSwerveController(
                                AutoConstants.kXController,
                                AutoConstants.kYController,
                                new PIDController(0, 0, 0)),
                        (ChassisSpeeds speeds) ->
                                swerve.drive(
                                        (!hasPiece
                                                        && hasTarget.getAsBoolean()
                                                        && swerve.getOdoPose().getX()
                                                                > (color == Alliance.Blue
                                                                        ? 5
                                                                        : FieldConstants
                                                                                        .kFieldLength
                                                                                - 8.75)
                                                        && swerve.getOdoPose().getX()
                                                                < (color == Alliance.Blue
                                                                        ? 8.75
                                                                        : FieldConstants
                                                                                        .kFieldLength
                                                                                - 5))
                                                ? autoDriveVelocities.get().dx * SLOWDOWN
                                                : speeds.vxMetersPerSecond * SLOWDOWN,
                                        (!hasPiece
                                                        && hasTarget.getAsBoolean()
                                                        && swerve.getOdoPose().getX()
                                                                > (color == Alliance.Blue
                                                                        ? 5
                                                                        : FieldConstants
                                                                                        .kFieldLength
                                                                                - 8.75)
                                                        && swerve.getOdoPose().getX()
                                                                < (color == Alliance.Blue
                                                                        ? 8.75
                                                                        : FieldConstants
                                                                                        .kFieldLength
                                                                                - 5))
                                                ? autoDriveVelocities.get().dy * SLOWDOWN
                                                : speeds.vyMetersPerSecond * SLOWDOWN,
                                        deeTheta()),
                        () -> mirror)
                .andThen(Commands.runOnce(() -> swerve.drive(0, 0, 0), swerve));
    }

    public Command followChoreoPathWithOverrideFastOnTheMove(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        PathPlannerLogging.logActivePath(PathPlannerPath.fromChoreoTrajectory(path));
        return customChoreoFolloweForOverride(
                        traj,
                        swerve::getOdoPose,
                        choreoSwerveController(
                                AutoConstants.kXController,
                                AutoConstants.kYController,
                                new PIDController(0, 0, 0)),
                        (ChassisSpeeds speeds) ->
                                swerve.drive(
                                        (!hasPiece
                                                        && hasTarget.getAsBoolean()
                                                        && swerve.getOdoPose().getX()
                                                                > (color == Alliance.Blue
                                                                        ? 5
                                                                        : FieldConstants
                                                                                        .kFieldLength
                                                                                - 8.75)
                                                        && swerve.getOdoPose().getX()
                                                                < (color == Alliance.Blue
                                                                        ? 8.75
                                                                        : FieldConstants
                                                                                        .kFieldLength
                                                                                - 5))
                                                ? autoDriveVelocities.get().dx * SLOWDOWN
                                                : speeds.vxMetersPerSecond * 0.9 * SLOWDOWN,
                                        (!hasPiece
                                                        && hasTarget.getAsBoolean()
                                                        && swerve.getOdoPose().getX()
                                                                > (color == Alliance.Blue
                                                                        ? 5
                                                                        : FieldConstants
                                                                                        .kFieldLength
                                                                                - 8.75)
                                                        && swerve.getOdoPose().getX()
                                                                < (color == Alliance.Blue
                                                                        ? 8.75
                                                                        : FieldConstants
                                                                                        .kFieldLength
                                                                                - 5))
                                                ? autoDriveVelocities.get().dy * SLOWDOWN
                                                : speeds.vyMetersPerSecond * 0.9 * SLOWDOWN,
                                        onTheMoveRotSupplier.getAsDouble()),
                        () -> mirror)
                .andThen(Commands.runOnce(() -> swerve.drive(0, 0, 0), swerve));
    }

    public Command followChoreoPathWithOverride(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        PathPlannerLogging.logActivePath(PathPlannerPath.fromChoreoTrajectory(path));
        Logger.recordOutput("Autos/current path", path);
        return customChoreoFolloweForOverrideSlow(
                        traj,
                        swerve::getOdoPose,
                        choreoSwerveController(
                                AutoConstants.kXController,
                                AutoConstants.kYController,
                                new PIDController(0, 0, 0)),
                        (ChassisSpeeds speeds) ->
                                swerve.drive(
                                        (!this.hasPiece && hasTarget.getAsBoolean())
                                                ? autoDriveVelocities.get().dx * SLOWDOWN
                                                : speeds.vxMetersPerSecond * 0.6 * SLOWDOWN,
                                        (!this.hasPiece && hasTarget.getAsBoolean())
                                                ? autoDriveVelocities.get().dy * SLOWDOWN
                                                : speeds.vyMetersPerSecond * 0.6 * SLOWDOWN,
                                        deeTheta()),
                        () -> mirror)
                .andThen(Commands.runOnce(() -> swerve.drive(0, 0, 0), swerve));
    }

    public Command followChoreoPathWithOverrideNoverrideFast(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        PathPlannerLogging.logActivePath(PathPlannerPath.fromChoreoTrajectory(path));
        Logger.recordOutput("Autos/current path", path);
        return customChoreoFolloweForOverrideSlow(
                        traj,
                        swerve::getOdoPose,
                        choreoSwerveController(
                                AutoConstants.kXController,
                                AutoConstants.kYController,
                                AutoConstants.kRotController),
                        (ChassisSpeeds speeds) -> {
                            var motionRotComponent = deeTheta();
                            var motionXComponent = speeds.vxMetersPerSecond;
                            var motionYComponent = speeds.vyMetersPerSecond;

                            var posexthresholdlow =
                                    color == Alliance.Blue ? 5 : FieldConstants.kFieldLength - 8.75;
                            var posexThresholdHigh =
                                    color == Alliance.Blue ? 8.75 : FieldConstants.kFieldLength - 5;

                            var isInPose =
                                    swerve.getOdoPose().getX() > posexthresholdlow
                                            && swerve.getOdoPose().getX() < posexThresholdHigh;

                            if (!this.hasPiece && hasTarget.getAsBoolean() && isInPose) {
                                motionXComponent = autoDriveVelocities.get().dx;
                                motionYComponent = autoDriveVelocities.get().dy;
                                motionRotComponent = autoDriveVelocities.get().dtheta;
                            }

                            if (isInPose) {
                                motionRotComponent = speeds.omegaRadiansPerSecond;
                            }
                            swerve.drive(motionXComponent, motionYComponent, motionRotComponent);
                        },
                        () -> mirror)
                .andThen(Commands.runOnce(() -> swerve.drive(0, 0, 0), swerve));
    }

    public Command followChoreoPathWithOverrideLongTimer(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        PathPlannerLogging.logActivePath(PathPlannerPath.fromChoreoTrajectory(path));
        Logger.recordOutput("Autos/current path", path);
        return customChoreoFolloweForOverrideLongTimer(
                        traj,
                        swerve::getOdoPose,
                        choreoSwerveController(
                                AutoConstants.kXController,
                                AutoConstants.kYController,
                                new PIDController(0, 0, 0)),
                        (ChassisSpeeds speeds) ->
                                swerve.drive(
                                        (!this.hasPiece && hasTarget.getAsBoolean())
                                                ? autoDriveVelocities.get().dx * SLOWDOWN
                                                : speeds.vxMetersPerSecond * 0.6 * SLOWDOWN,
                                        (!this.hasPiece && hasTarget.getAsBoolean())
                                                ? autoDriveVelocities.get().dy * SLOWDOWN
                                                : speeds.vyMetersPerSecond * 0.6 * SLOWDOWN,
                                        deeTheta()),
                        () -> mirror)
                .andThen(Commands.runOnce(() -> swerve.drive(0, 0, 0), swerve));
    }

    public Command customChoreoFolloweForOverride(
            ChoreoTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            ChoreoControlFunction controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            BooleanSupplier mirrorTrajectory) {
        var timer = new edu.wpi.first.wpilibj.Timer();
        return new FunctionalCommand(
                timer::restart,
                () -> {
                    PathPlannerLogging.logCurrentPose(swerve.getOdoPose());
                    PathPlannerLogging.logTargetPose(
                            trajectory
                                    .sample(timer.get(), mirrorTrajectory.getAsBoolean())
                                    .getPose());
                    ;
                    outputChassisSpeeds.accept(
                            controller.apply(
                                    poseSupplier.get(),
                                    trajectory.sample(
                                            timer.get(), mirrorTrajectory.getAsBoolean())));
                },
                (interrupted) -> {
                    timer.stop();
                    outputChassisSpeeds.accept(new ChassisSpeeds());
                },
                () ->
                        timer.hasElapsed(0.5)
                                && ((swerve.getVel() < 0.35
                                                & swerve.getOdoPose()
                                                                .minus(
                                                                        mirrorTrajectory
                                                                                                .getAsBoolean()
                                                                                        == false
                                                                                ? trajectory
                                                                                        .getFinalPose()
                                                                                : AllianceFlip
                                                                                        .flipForPP(
                                                                                                trajectory
                                                                                                        .getFinalPose()))
                                                                .getTranslation()
                                                                .getNorm()
                                                        < 0.4)
                                        || (currentYes.getAsBoolean())),
                swerve);
    }

    public Command customChoreoFolloweForOverrideLongTimer(
            ChoreoTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            ChoreoControlFunction controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            BooleanSupplier mirrorTrajectory) {
        var timer = new edu.wpi.first.wpilibj.Timer();
        return new FunctionalCommand(
                timer::restart,
                () -> {
                    PathPlannerLogging.logCurrentPose(swerve.getOdoPose());
                    PathPlannerLogging.logTargetPose(
                            trajectory
                                    .sample(timer.get(), mirrorTrajectory.getAsBoolean())
                                    .getPose());
                    ;
                    outputChassisSpeeds.accept(
                            controller.apply(
                                    poseSupplier.get(),
                                    trajectory.sample(
                                            timer.get(), mirrorTrajectory.getAsBoolean())));
                },
                (interrupted) -> {
                    timer.stop();
                    outputChassisSpeeds.accept(new ChassisSpeeds());
                },
                () ->
                        timer.hasElapsed(0.9) // gives the note more time to get in the intake;
                                && ((swerve.getVel() < 0.2
                                                & swerve.getOdoPose()
                                                                .minus(
                                                                        mirrorTrajectory
                                                                                                .getAsBoolean()
                                                                                        == false
                                                                                ? trajectory
                                                                                        .getFinalPose()
                                                                                : AllianceFlip
                                                                                        .flipForPP(
                                                                                                trajectory
                                                                                                        .getFinalPose()))
                                                                .getTranslation()
                                                                .getNorm()
                                                        < 0.3)
                                        || (currentYes.getAsBoolean())),
                swerve);
    }

    public Command customChoreoFolloweForOverrideSlow(
            ChoreoTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            ChoreoControlFunction controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            BooleanSupplier mirrorTrajectory) {
        var timer = new edu.wpi.first.wpilibj.Timer();
        return new FunctionalCommand(
                timer::restart,
                () -> {
                    PathPlannerLogging.logCurrentPose(swerve.getOdoPose());
                    PathPlannerLogging.logTargetPose(
                            trajectory
                                    .sample(timer.get(), mirrorTrajectory.getAsBoolean())
                                    .getPose());
                    ;
                    outputChassisSpeeds.accept(
                            controller.apply(
                                    poseSupplier.get(),
                                    trajectory.sample(
                                            timer.get(), mirrorTrajectory.getAsBoolean())));
                },
                (interrupted) -> {
                    timer.stop();
                    outputChassisSpeeds.accept(new ChassisSpeeds());
                },
                () ->
                        timer.hasElapsed(0.5)
                                && ((swerve.getVel() < 0.2
                                                & swerve.getOdoPose()
                                                                .minus(
                                                                        mirrorTrajectory
                                                                                                .getAsBoolean()
                                                                                        == false
                                                                                ? trajectory
                                                                                        .getFinalPose()
                                                                                : AllianceFlip
                                                                                        .flipForPP(
                                                                                                trajectory
                                                                                                        .getFinalPose()))
                                                                .getTranslation()
                                                                .getNorm()
                                                        < 0.3)
                                        || (currentYes.getAsBoolean())),
                swerve);
    }

    public static ChoreoControlFunction choreoSwerveController(
            PIDController xController,
            PIDController yController,
            PIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return (pose, referenceState) -> {
            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double rotationFF = referenceState.angularVelocity;

            double xFeedback = xController.calculate(pose.getX(), referenceState.x);
            double yFeedback = yController.calculate(pose.getY(), referenceState.y);
            double rotationFeedback =
                    rotationController.calculate(
                            pose.getRotation().getRadians(), referenceState.heading);

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback,
                    yFF + yFeedback,
                    rotationFF + rotationFeedback,
                    pose.getRotation());
        };
    }

    public double deeTheta() {
        return autoDriveVelocities.get().dtheta;
    }

    public Command followChoreoPath(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        return Choreo.choreoSwerveCommand(
                traj,
                swerve::getOdoPose,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                (ChassisSpeeds speeds) ->
                        swerve.drive(
                                speeds.vxMetersPerSecond,
                                speeds.vyMetersPerSecond,
                                speeds.omegaRadiansPerSecond),
                () -> mirror,
                swerve);
    }
}
