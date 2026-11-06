package examples;

import com.javiergs.turtle4j.RosbridgeConnection;
import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import javax.swing.*;

public class ReadScan {
    public static void main(String[] args) throws Exception {
        String endpoint = args.length > 0
                ? args[0] : "ws://192.168.185.3:9090";

        CountDownLatch received = new CountDownLatch(1);
        AtomicBoolean claimed = new AtomicBoolean(false);
        AtomicReference<List<Point2D.Double>> snapshot =
                new AtomicReference<>();

        try (RosbridgeConnection connection =
                     new RosbridgeConnection(endpoint)) {
            connection.connect();

            connection.subscribe("/scan", "sensor_msgs/msg/LaserScan", scan -> {
                // Capture exactly one message.
                if (!claimed.compareAndSet(false, true)) return;

                double angleMin = scan.path("angle_min").asDouble();
                double increment = scan.path("angle_increment").asDouble();
                double rangeMin = scan.path("range_min").asDouble();
                double rangeMax = scan.path("range_max").asDouble();
                var ranges = scan.path("ranges");

                List<Point2D.Double> points = new ArrayList<>();

                for (int i = 0; i < ranges.size(); i++) {
                    var sample = ranges.get(i);
                    if (!sample.isNumber()) continue;

                    double distance = sample.asDouble();
                    if (!Double.isFinite(distance)
                            || distance < rangeMin
                            || distance > rangeMax) {
                        continue;
                    }

                    double angle = angleMin + i * increment;
                    points.add(new Point2D.Double(
                            distance * Math.cos(angle),
                            distance * Math.sin(angle)));
                }

                System.out.printf("Frame: %s | Samples: %d | Valid: %d%n",
                        scan.path("header").path("frame_id").asText(),
                        ranges.size(), points.size());

                snapshot.set(List.copyOf(points));
                received.countDown();
            });

            if (!received.await(10, TimeUnit.SECONDS)) {
                System.out.println("No scan received within 10 seconds.");
                return;
            }
        }

        List<Point2D.Double> points = snapshot.get();
        if (points.isEmpty()) {
            System.out.println("The scan contained no valid points.");
            return;
        }

        // Keep main alive until the window closes, including under Maven.
        CountDownLatch closed = new CountDownLatch(1);

        SwingUtilities.invokeAndWait(() -> {
            JFrame frame = new JFrame("Lidar — single scan");
            frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
            frame.add(new ScanPanel(points));
            frame.addWindowListener(new WindowAdapter() {
                @Override
                public void windowClosed(WindowEvent event) {
                    closed.countDown();
                }
            });
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });

        closed.await();
    }

    private static final class ScanPanel extends JPanel {
        private final List<Point2D.Double> points;
        private final double extent;

        ScanPanel(List<Point2D.Double> points) {
            this.points = points;

            // Fit every point while preserving equal x/y scaling.
            double maximum = 1.0;
            for (Point2D.Double point : points) {
                maximum = Math.max(maximum,
                        Math.max(Math.abs(point.x), Math.abs(point.y)));
            }
            extent = maximum * 1.1;

            setPreferredSize(new Dimension(800, 800));
            setBackground(Color.WHITE);
        }

        @Override
        protected void paintComponent(Graphics graphics) {
            super.paintComponent(graphics);
            Graphics2D g = (Graphics2D) graphics.create();

            try {
                g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                        RenderingHints.VALUE_ANTIALIAS_ON);

                int centerX = getWidth() / 2;
                int centerY = getHeight() / 2;
                double scale = Math.max(1.0,
                        (Math.min(getWidth(), getHeight()) - 100)
                                / (2.0 * extent));

                // Grid lines spaced one meter apart.
                g.setColor(new Color(230, 230, 230));
                int meters = (int) Math.ceil(extent);
                for (int meter = -meters; meter <= meters; meter++) {
                    int x = (int) Math.round(centerX + meter * scale);
                    int y = (int) Math.round(centerY - meter * scale);
                    g.drawLine(x, 35, x, getHeight() - 35);
                    g.drawLine(35, y, getWidth() - 35, y);
                }

                g.setColor(Color.GRAY);
                g.drawLine(35, centerY, getWidth() - 35, centerY);
                g.drawLine(centerX, 35, centerX, getHeight() - 35);
                g.drawString("+x", getWidth() - 30, centerY - 8);
                g.drawString("+y", centerX + 8, 30);

                g.setColor(new Color(25, 100, 190));
                for (Point2D.Double point : points) {
                    int x = (int) Math.round(centerX + point.x * scale);
                    int y = (int) Math.round(centerY - point.y * scale);
                    g.fillOval(x - 2, y - 2, 4, 4);
                }

                // Red dot marks the scanner's origin.
                g.setColor(Color.RED);
                g.fillOval(centerX - 5, centerY - 5, 10, 10);

                g.setColor(Color.DARK_GRAY);
                g.drawString("Frozen scan | " + points.size()
                        + " points | Grid: 1 m | Frame: rplidar_link", 15, 20);
            } finally {
                g.dispose();
            }
        }
    }
}
