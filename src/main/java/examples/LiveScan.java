package examples;

import com.javiergs.turtle4j.RosbridgeConnection;
import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicReference;
import javax.swing.*;


/**
 * Live lidar visulaizer with a nearest obstacle readout.
 *
 * Unlike ReadScan, which freezes a single frame, this example
 * keeps the subscription open and repaints on every incoming
 * /scan message. It also reports the closest valid return
 * and its bearing, which is the trigger signal a reactive
 * obstacle-avoidance controller would act on.
 *
 * ARGS: endpoint, scanTopic , defaults="ws://localhost:9090"
 * /scan. Bearing convection: 0 degress is straight ahead (+x),
 * positive is counter-clockwise toward +y (left).
 */

public class LiveScan {
    public static void main(String[] args) throws Exception {
          String endpoint  = args.length > 0 ? args[0] : "ws://localhost:9090";
          String scanTopic = args.length > 1 ? args[1] : "/scan";

          // Shared, thread-safe handoff from the WebSocket thread to the EDT.
          AtomicReference<Frame> latest = new AtomicReference<>(Frame.empty());
          CountDownLatch closed = new CountDownLatch(1);

          RosbridgeConnection connection = new RosbridgeConnection(endpoint);
          connection.connect();

          ScanPanel[] panelRef = new ScanPanel[1];

          SwingUtilities.invokeAndWait(() -> {
              JFrame frame = new JFrame("Lidar — live");
              frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
              ScanPanel panel = new ScanPanel();
              panelRef[0] = panel;
              frame.add(panel);
              frame.addWindowListener(new WindowAdapter() {
                  @Override public void windowClosed(WindowEvent event) {
                      connection.close();
                      closed.countDown();
                  }
              });
              frame.pack();
              frame.setLocationRelativeTo(null);
              frame.setVisible(true);
          });

          connection.subscribe(scanTopic, "sensor_msgs/msg/LaserScan", scan -> {
              double angleMin  = scan.path("angle_min").asDouble();
              double increment = scan.path("angle_increment").asDouble();
              double rangeMin  = scan.path("range_min").asDouble();
              double rangeMax  = scan.path("range_max").asDouble();
              var ranges = scan.path("ranges");

              List<Point2D.Double> points = new ArrayList<>();
              double nearestDist = Double.POSITIVE_INFINITY;
              double nearestAngle = 0.0;

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

                  if (distance < nearestDist) {
                      nearestDist = distance;
                      nearestAngle = angle;
                  }
              }

              String frameId = scan.path("header").path("frame_id").asText();
              Frame f = new Frame(List.copyOf(points), nearestDist, nearestAngle, frameId);
              latest.set(f);
              SwingUtilities.invokeLater(() -> panelRef[0].setFrame(f));
          });

          closed.await();
      }

      /** Immutable snapshot of one scan, computed off the EDT. */
      private record Frame(List<Point2D.Double> points,
                           double nearestDist,
                           double nearestAngle,
                           String frameId) {
          static Frame empty() {
              return new Frame(List.of(), Double.POSITIVE_INFINITY, 0.0, "");
          }
          boolean hasNearest() { return Double.isFinite(nearestDist); }
      }

      private static final class ScanPanel extends JPanel {
          // Fixed view extent (meters) so the picture does not jitter frame to frame.
          private static final double EXTENT = 4.0;
          private volatile Frame frame = Frame.empty();

          ScanPanel() {
              setPreferredSize(new Dimension(800, 800));
              setBackground(Color.WHITE);
          }

          void setFrame(Frame f) {
              this.frame = f;
              repaint();
          }

          @Override
          protected void paintComponent(Graphics graphics) {
              super.paintComponent(graphics);
              Graphics2D g = (Graphics2D) graphics.create();
              try {
                  g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                          RenderingHints.VALUE_ANTIALIAS_ON);

                  Frame f = this.frame;
                  int centerX = getWidth() / 2;
                  int centerY = getHeight() / 2;
                  double scale = Math.max(1.0,
                          (Math.min(getWidth(), getHeight()) - 100) / (2.0 * EXTENT));

                  // Grid lines spaced one meter apart.
                  g.setColor(new Color(230, 230, 230));
                  int meters = (int) Math.ceil(EXTENT);
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

                  // Scan points.
                  g.setColor(new Color(25, 100, 190));
                  for (Point2D.Double point : f.points()) {
                      int x = (int) Math.round(centerX + point.x * scale);
                      int y = (int) Math.round(centerY - point.y * scale);
                      g.fillOval(x - 2, y - 2, 4, 4);
                  }

                  // Nearest-obstacle marker + line.
                  String readout;
                  if (f.hasNearest()) {
                      double nx = f.nearestDist() * Math.cos(f.nearestAngle());
                      double ny = f.nearestDist() * Math.sin(f.nearestAngle());
                      int px = (int) Math.round(centerX + nx * scale);
                      int py = (int) Math.round(centerY - ny * scale);
                      g.setColor(new Color(255, 140, 0));
                      g.drawLine(centerX, centerY, px, py);
                      g.fillOval(px - 5, py - 5, 10, 10);
                      readout = String.format("Nearest: %.2f m @ %+.0f°",
                              f.nearestDist(), Math.toDegrees(f.nearestAngle()));
                  } else {
                      readout = "Nearest: n/a";
                  }

                  // Scanner origin.
                  g.setColor(Color.RED);
                  g.fillOval(centerX - 5, centerY - 5, 10, 10);

                  g.setColor(Color.DARK_GRAY);
                  g.drawString("Live scan | " + f.points().size() + " points | Grid: 1 m | "
                          + readout + " | Frame: " + f.frameId(), 15, 20);
              } finally {
                  g.dispose();
              }
          }
      }
}
