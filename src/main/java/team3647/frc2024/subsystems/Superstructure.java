package team3647.frc2024.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2024.commands.ChurroCommands;
import team3647.frc2024.commands.IntakeCommands;
import team3647.frc2024.commands.KickerCommands;
import team3647.frc2024.commands.PivotCommands;
import team3647.frc2024.commands.ShooterCommands;
import team3647.frc2024.commands.WristCommands;
import team3647.frc2024.constants.ChurroConstants;
import team3647.frc2024.util.AutoDrive.DriveMode;

public class Superstructure {

    private final Intake intake;
    private final Kicker kicker;
    private final ShooterRight shooterRight;
    private final ShooterLeft shooterLeft;
    private final Pivot pivot;
    private final Wrist wrist;
    private final Churro churro;
    public final IntakeCommands intakeCommands;
    public final KickerCommands kickerCommands;
    public final ShooterCommands shooterCommands;
    public final PivotCommands pivotCommands;
    public final WristCommands wristCommands;
    public final ChurroCommands churroCommands;

    private final DoubleSupplier pivotAngleSupplier;
    private final DoubleSupplier shooterSpeedSupplierLeft;
    private final DoubleSupplier shooterSpeedSupplierRight;
    private final DoubleSupplier shooterSpeedThresholdSupplier;
    private final double pivotStowAngle = 40;
    private final double wristStowAngle = 100;
    private final double wristIntakeAngle = 0;
    private final double churroDeployAngle = 70;
    private final double churroStowAngle = ChurroConstants.kInitialDegree;
    private final double shootSpeed;
    public double currentLimit = 49;

    
    private boolean hasPiece = false;
    private boolean isClimbing;
    private boolean isIntaking = false;

    private final BooleanSupplier swerveAimed;

    private final BooleanSupplier feedShot;

    private final BooleanSupplier dontShoot;

    private final Trigger front;

    private DriveMode wantedShootingMode = DriveMode.SHOOT_STATIONARY;

    public Superstructure(
            Intake intake,
            Kicker kicker,
            ShooterRight shooterRight,
            ShooterLeft shooterLeft,
            Pivot pivot,
            Wrist wrist,
            Churro churro,
            DoubleSupplier pivotAngleSupplier,
            DoubleSupplier shooterSpeedSupplerLeft,
            DoubleSupplier shooterSpeedSupplerRight,
            DoubleSupplier shooterSpeedThresholdSupplier,
            double shootSpeed,
            BooleanSupplier feedShot,
            BooleanSupplier swerveAimed,
            BooleanSupplier dontShoot) {
        this.intake = intake;
        this.kicker = kicker;
        this.shooterRight = shooterRight;
        this.shooterLeft = shooterLeft;
        this.pivot = pivot;
        this.churro = churro;
        this.pivotAngleSupplier = pivotAngleSupplier;
        this.shooterSpeedSupplierLeft = shooterSpeedSupplerLeft;
        this.shooterSpeedSupplierRight = shooterSpeedSupplerRight;
        this.shooterSpeedThresholdSupplier = shooterSpeedThresholdSupplier;
        this.shootSpeed = shootSpeed;
        this.wrist = wrist;
        this.swerveAimed = swerveAimed;
        this.feedShot = feedShot;
        this.dontShoot = dontShoot;

        intakeCommands = new IntakeCommands(intake);
        kickerCommands = new KickerCommands(kicker);
        shooterCommands = new ShooterCommands(shooterRight, shooterLeft);
        pivotCommands = new PivotCommands(pivot);
        wristCommands = new WristCommands(wrist);
        churroCommands = new ChurroCommands(churro);

        front = new Trigger(() -> !pivot.frontPiece()).debounce(0.04);
    }

    public Command feed() {
        return kickerCommands.kick();
    }

    public Command setIsIntaking() {
        return Commands.runOnce(() -> this.isIntaking = true);
    }

    public Command setIsNotIntaking() {
        return Commands.runOnce(() -> this.isIntaking = false);
    }

    public Command currentUp() {
        return Commands.runOnce(() -> this.currentLimit += 1);
    }

    public Command currentDown() {
        return Commands.runOnce(() -> this.currentLimit -= 1);
    }

    public Command setIsClimbing() {
        return Commands.runOnce(() -> this.isClimbing = true);
    }

    public Command setIsNotClimbing() {
        return Commands.runOnce(() -> this.isClimbing = false);
    }

    public boolean getIsIntaking() {
        return this.isIntaking;
    }

    public boolean isClimbing() {
        return isClimbing;
    }

    public Command spinUp() {
        return shooterCommands.setVelocityIndep(
                () -> shooterSpeedSupplierRight.getAsDouble() + 1,
                () -> shooterSpeedSupplierLeft.getAsDouble());
    }

    public Command spinUpPreload() {
        return shooterCommands.setVelocityIndep(() -> 3, () -> 3);
    }

    public Command spinUpTrap() {
        return shooterCommands.setVelocity(() -> 7, () -> 1.1);
    }

    public Command deployChurro() {
        return Commands.sequence(
                churroCommands
                        .setAngle(churroDeployAngle)
                        .until(() -> churro.angleReached(churroDeployAngle, 5)),
                churroCommands.setAngleSpringy(churroDeployAngle));
    }

    public Command stowChurro() {
        return churroCommands
                .setAngle(churroStowAngle)
                .until(() -> churro.angleReached(churroStowAngle, 3));
    }

    public Command spinUpAmp() {
        return shooterCommands.setVelocity(() -> 10, () -> 1);
    }

    public Command setShootModeStationary() {
        return Commands.runOnce(() -> this.wantedShootingMode = DriveMode.SHOOT_STATIONARY);
    }

    public Command setShootModeMoving() {
        return Commands.runOnce(() -> this.wantedShootingMode = DriveMode.SHOOT_ON_THE_MOVE);
    }

    public Command tinyPivot() {
        return pivotCommands.setAngle(() -> 11);
    }

    public Command setShootModeClean() {
        return Commands.runOnce(() -> this.wantedShootingMode = DriveMode.CLEAN);
    }

    public Command setShootModeFeed() {
        return Commands.runOnce(() -> this.wantedShootingMode = DriveMode.FEED);
    }

    public DriveMode getWantedShootingMode() {
        return this.wantedShootingMode;
    }

    public double getDesiredSpeed() {
        return shootSpeed;
    }

    public double getWantedPivot() {
        return pivotAngleSupplier.getAsDouble();
    }

    public boolean getPiece() {
        return hasPiece;
    }

    public Command stowAll() {
        return Commands.parallel(
                wristCommands
                        .setAngle(wristStowAngle)
                        .until(() -> wrist.angleReached(wristStowAngle, 5)),
                Commands.parallel(
                        pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble()),
                        shooterCommands.kill(),
                        kickerCommands.kill(),
                        intakeCommands.kill(),
                        stowChurro()));
    }

    public boolean currentYes() {
        return intake.getMasterCurrent() > currentLimit && wrist.getAngle() < 5; // 41
    }

    public boolean current() {
        return intake.getMasterCurrent() > currentLimit;
    }

    public Command setPiece() {
        return Commands.runOnce(() -> this.hasPiece = true);
    }

    public Command ejectPiece() {
        return Commands.runOnce(() -> this.hasPiece = false);
    }

    public boolean pivotReady() {
        return pivot.angleReached(pivotAngleSupplier.getAsDouble(), 3);
    }

    public boolean swerveReady() {
        return swerveAimed.getAsBoolean();
    }

    public boolean readyForShot() {
        return aimedAtSpeaker() && !dontShoot.getAsBoolean();
    }

    public Command shoot() {
        return Commands.parallel(
                prep(), spinUp()
                // Commands.sequence(
                //         // Commands.waitSeconds(2.5),
                //         Commands.waitUntil(
                //                         () ->
                //                                 shooterLeft.velocityReached(30, 2)
                //                                         && pivot.angleReached(
                //
                // pivotAngleSupplier.getAsDouble(),
                // 5)
                //                                         && swerveAimed.getAsBoolean())
                //                 .withTimeout(1.2),
                //         feed())
                );
    }

    public Command cleanShoot() {
        return Commands.parallel(
                prep(), spinUp(), kickerCommands.fastKick()
                // Commands.sequence(
                //         // Commands.waitSeconds(2.5),
                //         Commands.waitUntil(
                //                         () ->
                //                                 shooterLeft.velocityReached(30, 2)
                //                                         && pivot.angleReached(
                //                                                 pivotAngleSupplier.getAsDouble(),
                // 5)
                //                                         && swerveAimed.getAsBoolean())
                //                 .withTimeout(1.2),
                //         feed())
                );
    }

    public boolean aimedAtSpeaker() {
        return shooterRight.velocityGreater(shooterSpeedThresholdSupplier.getAsDouble())
                && pivot.angleReached(pivotAngleSupplier.getAsDouble(), 5)
                && swerveAimed.getAsBoolean();
    }

    public Command shootAmp(DoubleSupplier swerveY) {
        return Commands.parallel(
                prepAmp(), spinUpAmp()
                // Commands.sequence(
                //         // Commands.waitSeconds(2.5),
                //         Commands.waitUntil(
                //                         () ->
                //                                 shooterLeft.velocityReached(
                //                                                 shootSpeed
                //                                                         * 2
                //                                                         /
                // getDesiredSpeed()
                //                                                         *
                // ShooterConstants
                //
                // .kLeftRatio,
                //                                                 1)
                //                                         && pivot.angleReached(55, 5)
                //                                         && swerveAimed.getAsBoolean())
                //                 .withTimeout(0.7),
                //         feed())
                );
    }

    public Command batterShot() {
        return Commands.parallel(
                batterPrep(), spinUp()
                // Commands.sequence(
                //         // Commands.waitSeconds(2.5),
                //         Commands.waitUntil(
                //                         () ->
                //                                 shooterLeft.velocityReached20, 1)
                //                                         && pivot.angleReached(60, 5))
                //                 .withTimeout(0.5),
                //         feed())
                );
    }

    public Command shootManual() {
        return Commands.parallel(
                prepManual(),
                spinUp(),
                Commands.sequence(
                        Commands.waitSeconds(2), Commands.parallel(intake(), ejectPiece())));
    }

    public Command trapShot(DoubleSupplier swerveX) {
        return Commands.sequence(
                Commands.deadline(
                                Commands.waitSeconds(1), prepTrap(), spinUpTrap()
                                // Commands.sequence(
                                //         // Commands.waitSeconds(2.5),
                                //         Commands.waitUntil(
                                //                         () ->
                                //                                 shooterLeft.velocityReached20, 1)
                                //                                         && pivot.angleReached(60,
                                // 5))
                                //                 .withTimeout(0.5),
                                //         feed())
                                )
                        .andThen(stowFromTrapShoot()));
    }

    public Command shootStow() {
        return shoot().andThen(Commands.waitSeconds(0.5)).andThen(stowFromShoot());
    }

    public Command stowFromTrapShoot() {
        return Commands.sequence(
                        Commands.parallel(prepTrap(), spinUpTrap(), feed()).withTimeout(0.6),
                        Commands.parallel(
                                        pivotCommands.setAngle(
                                                () -> pivotAngleSupplier.getAsDouble()),
                                        shooterCommands.kill(),
                                        kickerCommands.kill(),
                                        ejectPiece())
                                .withTimeout(0.1))
                .andThen(ejectPiece());
    }

    public Command prep() {
        return pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble());
    }

    public Command prepTrap() {
        return pivotCommands.setAngle(() -> 70);
    }

    public Command prepAmp() {
        return pivotCommands.setAngle(() -> 50);
    }

    public Command batterPrep() {
        return pivotCommands.setAngle(() -> 62);
    }

    public Command prepManual() {
        return pivotCommands.setAngle(() -> SmartDashboard.getNumber("pivot interp angle", 40));
    }

    public Command stowFromShoot() {
        return Commands.sequence(
                Commands.parallel(prep(), spinUp(), feed()).withTimeout(0.6),
                Commands.parallel(
                                pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble()),
                                shooterCommands.kill(),
                                kickerCommands.kill())
                        .withTimeout(0.1));
    }

    public Command stowNoShoot() {
        return Commands.parallel(
                        pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble()),
                        shooterCommands.kill(),
                        kickerCommands.kill())
                .withTimeout(0.1);
    }

    public Command stowFromAmpShoot() {
        return Commands.sequence(
                Commands.parallel(
                                deployChurro(),
                                prepAmp(),
                                spinUpAmp(),
                                Commands.sequence(Commands.waitSeconds(0.3), feed()))
                        .withTimeout(1),
                Commands.deadline(
                        stowChurro(),
                        pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble()),
                        shooterCommands.kill(),
                        kickerCommands.kill()));
    }

    public boolean hasPiece() {
        return getPiece() && (frontPiece() || pivot.backPiece());
    }

    public Command stowFromBatterShoot() {
        return Commands.sequence(
                Commands.parallel(batterPrep(), spinUp(), feed()).withTimeout(0.6),
                Commands.parallel(
                                pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble()),
                                shooterCommands.kill(),
                                kickerCommands.kill())
                        .withTimeout(0.1));
    }

    public Command intake() {
        return Commands.parallel(wristCommands.setAngle(wristIntakeAngle), intakeCommands.intake());
    }

    public Command intake(BooleanSupplier hasPiece) {
        return Commands.parallel(
                wristCommands.setAngle(wristIntakeAngle),
                Commands.sequence(
                        Commands.waitUntil(() -> !hasPiece.getAsBoolean()),
                        intakeCommands.intake()));
    }

    public Command wristDown() {
        return wristCommands.setAngle(wristIntakeAngle);
    }

    public Command wristUp() {
        return wristCommands.setAngle(wristStowAngle);
    }

    public Command passToShooter() {
        return Commands.parallel(
                        setIsIntaking(),
                        intakeCommands.kill(),
                        kickerCommands.kick(),
                        // pivotCommands.setAngle(() -> 20),
                        wristCommands.setAngle(() -> 110).until(() -> wrist.angleReached(110, 5)))
                .withTimeout(0.4)
                .andThen(shootThrough());
    }

    public Command passToShooterClean() {
        return Commands.parallel(
                        setIsIntaking(),
                        intakeCommands.kill(),
                        // pivotCommands.setAngle(() -> 20),
                        wristCommands.setAngle(() -> 110).until(() -> wrist.angleReached(110, 5)))
                .withTimeout(0.3)
                .andThen(shootThroughClean());
    }

    public Command passToShooterNoKicker(Trigger shouldGO) {
        return Commands.parallel(
                        intakeCommands.kill(),
                        wristCommands.setAngle(() -> wrist.getInverseKinematics(pivot.getAngle())))
                .until(
                        shouldGO.and(
                                () ->
                                        wrist.angleReached(
                                                wrist.getInverseKinematics(pivot.getAngle()), 5)))
                .andThen(Commands.deadline(shootThroughNoKicker(), spinUp()));
    }

    public Command shootThrough() {
        return Commands.parallel(intakeCommands.intake(), kickerCommands.fastKick())
                // pivotCommands.setAngle(() -> 20))
                .until(() -> pivot.frontPiece())
                .andThen(slightReverse().until(front))
                // .withTimeout(1)
                .andThen(
                        Commands.deadline(stowIntake(), setIsNotIntaking(), kickerCommands.kill()));
    }

    public Command shootThroughClean() {
        return Commands.parallel(intakeCommands.intake())
                // pivotCommands.setAngle(() -> 20))
                .until(() -> pivot.frontPiece())
                // .withTimeout(1)
                .andThen(Commands.deadline(stowIntake(), setIsNotIntaking()));
    }

    public Command sourceIntake() {
        return Commands.parallel(
                        pivotCommands.setAngle(() -> 60),
                        kickerCommands.unkick(),
                        shooterCommands.setVelocity(() -> -8, () -> 1))
                .until(() -> pivot.backPiece())
                .andThen(stowFromSourceIntake());
    }

    public Command stowFromSourceIntake() {
        return Commands.parallel(prep(), kickerCommands.kill(), shooterCommands.kill());
    }

    public Command shootThroughNoKicker() {
        return intakeCommands
                .intake()
                .until(() -> pivot.backPiece())
                .withTimeout(0.6)
                .andThen(stowIntakeNoPivot());
    }

    public Command stowIntakeNoPivot() {
        return Commands.parallel(
                wristCommands
                        .setAngle(wristStowAngle)
                        .until(() -> wrist.angleReached(wristStowAngle, 5)),
                intakeCommands.kill());
    }

    public Command stowIntake() {
        return Commands.parallel(
                prep(),
                wristCommands
                        .setAngle(wristStowAngle)
                        .until(() -> wrist.angleReached(wristStowAngle, 5)),
                intakeCommands.kill());
    }

    public Command stowIntakeNoPrep() {
        return Commands.parallel(
                wristCommands
                        .setAngle(wristStowAngle)
                        .until(() -> wrist.angleReached(wristStowAngle, 5)),
                intakeCommands.kill());
    }

    public boolean wristAtStow() {
        return wrist.angleReached(wristStowAngle, 5);
    }

    public Command autoFeed(BooleanSupplier goodToGo) {
        return Commands.run(
                () -> {
                    if (goodToGo.getAsBoolean() && pivot.backPiece()) {
                        Commands.sequence(
                                Commands.waitUntil(
                                        () ->
                                                pivot.angleReached(
                                                        pivotAngleSupplier.getAsDouble(), 2)),
                                kickerCommands.fastKick());
                    } else if (!frontPiece() && !pivot.backPiece()) {
                        slightForwards();
                    } else {
                        kickerCommands.kill();
                    }
                });
    }

    public Command fastFeed() {
        return kickerCommands.fastKick();
    }

    public Command index() {
        if (pivot.frontPiece()) {
            return slightReverse().until(() -> !pivot.frontPiece());
        } else {
            return slightForwards().until(() -> pivot.frontPiece());
        }
    }

    public Command stowIntakeAndIndex() {
        return Commands.parallel(stowIntake(), index()).withTimeout(0.2);
    }

    public boolean frontPiece() {
        return pivot.frontPiece();
    }

    public Command slightForwards() {
        return kickerCommands.slowFeed();
    }

    public Command slightReverse() {
        return kickerCommands.unkick();
    }
}
