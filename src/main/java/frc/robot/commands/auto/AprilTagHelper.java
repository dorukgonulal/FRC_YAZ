package frc.robot.commands.auto;


import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import frc.robot.Robot;
import frc.robot.RobotContainer;


import frc.robot.commands.auto.AlignToReef;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;

public class AprilTagHelper {


    public enum FieldBranch{
        A(BranchSide.LEFT, ReefSide.ONE),
        B(BranchSide.RIGHT, ReefSide.ONE),
        C(BranchSide.LEFT, ReefSide.TWO),
        D(BranchSide.RIGHT, ReefSide.TWO),
        E(BranchSide.LEFT, ReefSide.THREE),
        F(BranchSide.RIGHT, ReefSide.THREE),
        G(BranchSide.LEFT, ReefSide.FOUR),
        H(BranchSide.RIGHT, ReefSide.FOUR),
        I(BranchSide.LEFT, ReefSide.FIVE),
        J(BranchSide.RIGHT, ReefSide.FIVE),
        K(BranchSide.LEFT, ReefSide.SIX),
        L(BranchSide.RIGHT, ReefSide.SIX);

        public SimpleBranch simpleBranchInfo;

        private FieldBranch(BranchSide branchSide, ReefSide reefSide) {
            this.simpleBranchInfo = new SimpleBranch(branchSide, reefSide);
        }
    }

    public record SimpleBranch(BranchSide branchSide, ReefSide reefSide) {
        public SimpleBranch mirror(){
            //TODO check if mirroring the branchside does work here
            return new SimpleBranch(branchSide.mirror(), reefSide.mirror());
        }
    }

    // X = side to side, Y = away from tag
    public enum BranchSide{ //? you could consider bringing the tag offsets back and modifying dynamics
        LEFT(new Translation2d(-0.153209 + 0.0381, 0.5406845 + 0.02 - 0.03175)), //-0.109236, 0.5406845 + 0.02))
        RIGHT(new Translation2d(0.218062 - 0.0508 + 0.01, 0.5408565 + 0.02 - 0.03175)),//0.218918 - 0.0508, 0.5408565 + 0.02))
        MIDDLE(new Translation2d(0.064853 - 0.03175, 0.5408565 + 0.02 + 0.0254));

        public Translation2d tagOffset;
        private BranchSide(Translation2d offsets) {
            tagOffset = offsets;
        }

        public BranchSide mirror(){
            switch (this) {
                case LEFT: return RIGHT;
                case MIDDLE: return MIDDLE;
                default: return LEFT;
            }
        }
    }

    public enum ReefSide{
        ONE(18, 7),
        SIX(19, 6),
        FIVE(20, 11),
        FOUR(21, 10),
        THREE(22, 9),
        TWO(17, 8);

        public final Pose2d redTagPose;
        public final Pose2d blueTagPose;

        public Pose2d getCurrent(){
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
                blueTagPose : 
                redTagPose;
        }


        public ReefSide mirror(){
            switch (this) {
                case ONE: return ONE;
                case TWO: return SIX;
                case THREE: return FIVE;
                case FOUR: return FOUR;
                case FIVE: return THREE;
                default: return TWO; //SIX case
            }
        }

        private ReefSide(int blue, int red) {
            var layout = RobotContainer.getFieldLayout();


            redTagPose =layout.getTagPose(red).get().toPose2d();
            blueTagPose = layout.getTagPose(blue).get().toPose2d();
        }
    } 

    public enum StationSide{
        LEFT,
        RIGHT;
    }





}