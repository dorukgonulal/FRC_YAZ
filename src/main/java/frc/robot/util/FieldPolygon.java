package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import java.util.Arrays;
import java.util.List;
import java.util.logging.Logger;

/**
 * FieldPolygon sınıfı:
 * 1) Robotun saha içindeki bölgesini tespit eder.
 * 2) Bölge sınırlarını hem SmartDashboard(Field2d) hem de
 * AdvantageScope(Odometry) üzerinden görselleştirir.
 */
public class FieldPolygon {
    private static final Logger logger = Logger.getLogger(FieldPolygon.class.getName());

    // SmartDashboard / 2D saha widget
    private final Field2d m_field = new Field2d();

    // Field2d üzerinde çizilecek alan objeleri
    private final FieldObject2d yellowArea;
    private final FieldObject2d greenArea;
    private final FieldObject2d blueArea;
    private final FieldObject2d pinkArea;
    private final FieldObject2d redArea;
    private final FieldObject2d orangeArea;

    // AdvantageScope için StructArrayPublisher<Translation2d>
    private final StructArrayPublisher<Translation2d> yellowPub;
    private final StructArrayPublisher<Translation2d> greenPub;
    private final StructArrayPublisher<Translation2d> bluePub;
    private final StructArrayPublisher<Translation2d> pinkPub;
    private final StructArrayPublisher<Translation2d> redPub;
    private final StructArrayPublisher<Translation2d> orangePub;

    public FieldPolygon() {
        // 1) Field2d'i NT'ye yayınla
        SmartDashboard.putData("Field", m_field);

        // 2) FieldObject2d objelerini oluştur
        yellowArea = m_field.getObject("Sarı Alan");
        greenArea = m_field.getObject("Yeşil Alan");
        blueArea = m_field.getObject("Mavi Alan");
        pinkArea = m_field.getObject("Pembe Alan");
        redArea = m_field.getObject("Kırmızı Alan");
        orangeArea = m_field.getObject("Turuncu Alan");

        // 3) StructArrayPublisher'ları başlat
        var nt = NetworkTableInstance.getDefault();
        yellowPub = nt.getStructArrayTopic("ZonePolygons/Yellow", Translation2d.struct).publish();
        greenPub = nt.getStructArrayTopic("ZonePolygons/Green", Translation2d.struct).publish();
        bluePub = nt.getStructArrayTopic("ZonePolygons/Blue", Translation2d.struct).publish();
        pinkPub = nt.getStructArrayTopic("ZonePolygons/Pink", Translation2d.struct).publish();
        redPub = nt.getStructArrayTopic("ZonePolygons/Red", Translation2d.struct).publish();
        orangePub = nt.getStructArrayTopic("ZonePolygons/Orange", Translation2d.struct).publish();

        // 4) Saha objelerini ve publisher'ı veri ile doldur
        yellowArea.setPoses(toPoseList(getYellowPoints()));
        yellowPub.set(getYellowTranslations());

        greenArea.setPoses(toPoseList(getGreenPoints()));
        greenPub.set(getGreenTranslations());

        blueArea.setPoses(toPoseList(getBluePoints()));
        bluePub.set(getBlueTranslations());

        pinkArea.setPoses(toPoseList(getPinkPoints()));
        pinkPub.set(getPinkTranslations());

        redArea.setPoses(toPoseList(getRedPoints()));
        redPub.set(getRedTranslations());

        orangeArea.setPoses(toPoseList(getOrangePoints()));
        orangePub.set(getOrangeTranslations());

        logger.info("Field polygons initialized and published to SmartDashboard and AdvantageScope.");
    }

    /**
     * Ray-casting algoritması: (x,y) nokta çokgen içinde mi?
     */
    public static boolean pointInPolygon(double x, double y, List<Point> poly) {
        boolean inside = false;
        int n = poly.size();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            Point pi = poly.get(i);
            Point pj = poly.get(j);
            if (((pi.y > y) != (pj.y > y))
                    && (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y) + pi.x)) {
                inside = !inside;
            }
        }
        return inside;
    }

    /**
     * RobotPose'a göre hangi alanda?
     * 
     * @return 1=Sarı, 2=Yeşil, 3=Mavi, 4=Pembe, 5=Kırmızı, 6=Turuncu, 0=diğer
     */
    public int checkRobotPosition(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        if (x < 0 || x > 8.768 || y < 0 || y > 8) {
            return 0;
        }
        if (pointInPolygon(x, y, getYellowPoints()))
            return 1;
        if (pointInPolygon(x, y, getGreenPoints()))
            return 2;
        if (pointInPolygon(x, y, getBluePoints()))
            return 3;
        if (pointInPolygon(x, y, getPinkPoints()))
            return 4;
        if (pointInPolygon(x, y, getRedPoints()))
            return 5;
        if (pointInPolygon(x, y, getOrangePoints()))
            return 6;
        return 0;
    }

    // Bölge köşeleri: meter cinsinden x/y
    private List<Point> getYellowPoints() {
        return Arrays.asList(
                new Point(4.501, 4.000), new Point(4.501, 8.000),
                new Point(8.768, 8.000), new Point(8.768, 6.176));
    }

    private List<Point> getGreenPoints() {
        return Arrays.asList(
                new Point(4.501, 4.000), new Point(8.768, 1.902),
                new Point(8.768, 6.176));
    }

    private List<Point> getBluePoints() {
        return Arrays.asList(
                new Point(4.501, 4.000), new Point(4.501, 0.000),
                new Point(8.768, 0.000), new Point(8.768, 1.902));
    }

    private List<Point> getPinkPoints() {
        return Arrays.asList(
                new Point(4.501, 4.000), new Point(0.000, 1.240),
                new Point(1.674, 0.000), new Point(4.501, 0.000));
    }

    private List<Point> getRedPoints() {
        return Arrays.asList(
                new Point(4.501, 4.000), new Point(0.000, 6.760),
                new Point(0.000, 1.240));
    }

    private List<Point> getOrangePoints() {
        return Arrays.asList(
                new Point(4.501, 4.000), new Point(0.000, 6.760),
                new Point(1.674, 8.000), new Point(4.501, 8.000));
    }

    // Field2d için Pose2d listesine dönüştürücü
    private List<Pose2d> toPoseList(List<Point> pts) {
        return pts.stream()
                .map(p -> new Pose2d(p.x, p.y, new Rotation2d()))
                .toList();
    }

    // StructArrayPublisher için Translation2d dizisi
    private Translation2d[] getYellowTranslations() {
        return getYellowPoints().stream()
                .map(p -> new Translation2d(p.x, p.y))
                .toArray(Translation2d[]::new);
    }

    private Translation2d[] getGreenTranslations() {
        return getGreenPoints().stream()
                .map(p -> new Translation2d(p.x, p.y))
                .toArray(Translation2d[]::new);
    }

    private Translation2d[] getBlueTranslations() {
        return getBluePoints().stream()
                .map(p -> new Translation2d(p.x, p.y))
                .toArray(Translation2d[]::new);
    }

    private Translation2d[] getPinkTranslations() {
        return getPinkPoints().stream()
                .map(p -> new Translation2d(p.x, p.y))
                .toArray(Translation2d[]::new);
    }

    private Translation2d[] getRedTranslations() {
        return getRedPoints().stream()
                .map(p -> new Translation2d(p.x, p.y))
                .toArray(Translation2d[]::new);
    }

    private Translation2d[] getOrangeTranslations() {
        return getOrangePoints().stream()
                .map(p -> new Translation2d(p.x, p.y))
                .toArray(Translation2d[]::new);
    }

    /** Temel x/y koordinat çifti */
    static class Point {
        final double x, y;

        Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}