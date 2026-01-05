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
@SuppressWarnings("unused")

/**
 * Bu sınıf, Reef üzerindeki hedeflere (Branch) hizalanmak için gerekli olan
 * geometrik hesaplamaları, offset (kaydırma) değerlerini ve mantıksal
 * eşleştirmeleri tutar.
 *  */
public class AprilTagHelper {

    /**
     * Reef üzerindeki her bir direği (Branch) temsil eden harfler (A'dan L'ye).
     * Her harf, Reef'in bir yüzünü (ReefSide) ve o yüzdeki tarafı (Left/Right)
     * belirtir.
     * 
     * Örnek: 'A' harfi -> 1. Yüzün (ReefSide.ONE) SOL tarafındaki (BranchSide.LEFT)
     * direktir.
     */
    public enum FieldBranch {
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

    /**
     * Bir Branch'in hangi yön (Sol/Sağ) ve hangi Reef yüzünde olduğunu bir arada
     * tutar.
     */
    public record SimpleBranch(BranchSide branchSide, ReefSide reefSide) {
        public SimpleBranch mirror() {
            // TODO check if mirroring the branchside does work here
            return new SimpleBranch(branchSide.mirror(), reefSide.mirror());
        }
    }

    /**
     * AprilTag'e GÖRE robotun durması gereken konumu ifade eder.
     * Translation2d değerleri (X, Y):
     * - X: Tag'e göre yatay kaydırma (Negatif: Sol, Pozitif: Sağ)
     * - Y: Tag'den uzaklık (Robot tag'in ne kadar önünde durmalı?)
     */
    public enum BranchSide { // ? you could consider bringing the tag offsets back and modifying dynamics
  
        LEFT(new Translation2d(-0.153209 + 0.0381, 0.5406845 + 0.02 + 0.0254)),


        RIGHT(new Translation2d(0.218062 - 0.0508 + 0.01, 0.5408565 + 0.02 + 0.0254)),

        MIDDLE(new Translation2d(0.064853 - 0.03175, 0.5408565 + 0.02 + 0.0254));

        public Translation2d tagOffset;

        private BranchSide(Translation2d offsets) {
            tagOffset = offsets;
        }

        public BranchSide mirror() {
            switch (this) {
                case LEFT:
                    return RIGHT;
                case MIDDLE:
                    return MIDDLE;
                default:
                    return LEFT;
            }
        }
    }

    /**
     * Reef'in 6 yüzünü ve her yüzdeki AprilTag ID'lerini tanımlar.
     * Her yüzün bir Mavi ID'si bir de Kırmızı ID'si vardır.
     */
    public enum ReefSide {
        ONE(18, 7), // 1. Yüz: Mavi 18, Kırmızı 7
        SIX(19, 6), // 6. Yüz
        FIVE(20, 11), // 5. Yüz
        FOUR(21, 10), // 4. Yüz
        THREE(22, 9), // 3. Yüz
        TWO(17, 8); // 2. Yüz

        public final Pose2d redTagPose;
        public final Pose2d blueTagPose;

        /**
         * Çalışan ittifak rengine göre doğru Tag'in pozisyonunu döndürür.
         * Eğer biz Maviysek Mavi tag'i, Kırmızıysak Kırmızı tag'i verir.
         */
        public Pose2d getCurrent() {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? blueTagPose : redTagPose;
        }

        // Yüzün simetriğini bulur (Sahayı ikiye katlarsan hangi yüze denk gelir?)
        public ReefSide mirror() {
            switch (this) {
                case ONE:
                    return ONE;
                case TWO:
                    return SIX;
                case THREE:
                    return FIVE;
                case FOUR:
                    return FOUR;
                case FIVE:
                    return THREE;
                default:
                    return TWO; // SIX case
            }
        }

        private ReefSide(int blue, int red) {
            var layout = RobotContainer.getFieldLayout();

            // FieldLayout'tan bu ID'lerin sahadaki gerçek (Pose2d) konumlarını çeker!
            redTagPose = layout.getTagPose(red).get().toPose2d();
            blueTagPose = layout.getTagPose(blue).get().toPose2d();
        }
    }

    public enum StationSide {
        LEFT,
        RIGHT;
    }

}