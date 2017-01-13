package us.ihmc.plotting.jfree;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Point2d;

import org.jfree.chart.ChartPanel;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class GraphingUtilityDemo
{
   public static void main(String[] args)
   {
      ArrayList<XYSeries> data = new ArrayList<XYSeries>();

      ArrayList<Point2d> series1 = new ArrayList<Point2d>();
      series1.add(new Point2d(3.0, 19.0));
      series1.add(new Point2d(6.0, 22.0));
      series1.add(new Point2d(9.0, 27.0));
      series1.add(new Point2d(12.0, 31.0));
      series1.add(new Point2d(15.0, 28.0));
      series1.add(new Point2d(18.0, 22.0));
      series1.add(new Point2d(21.0, 18.0));
      XYSeries series1Set = GraphingUtility.createXYSeries("0 Acceleration", series1);
      data.add(series1Set);

      ArrayList<Point2d> series2 = new ArrayList<Point2d>();
      series2.add(new Point2d(3.0, 18.0));
      series2.add(new Point2d(6.0, 20.0));
      series2.add(new Point2d(9.0, 22.0));
      series2.add(new Point2d(12.0, 24.0));
      series2.add(new Point2d(15.0, 31.0));
      series2.add(new Point2d(18.0, 28.0));
      series2.add(new Point2d(21.0, 21.0));
      XYSeries series2Set = GraphingUtility.createXYSeries("-1 Acceleration", series2);
      data.add(series2Set);

      ArrayList<Point2d> series3 = new ArrayList<Point2d>();
      series3.add(new Point2d(3.0, 20.0));
      series3.add(new Point2d(6.0, 23.0));
      series3.add(new Point2d(9.0, 31.0));
      series3.add(new Point2d(12.0, 26.0));
      series3.add(new Point2d(15.0, 21.0));
      series3.add(new Point2d(18.0, 19.0));
      series3.add(new Point2d(21.0, 17.0));
      XYSeries series3Set = GraphingUtility.createXYSeries("+1 Acceleration", series3);
      data.add(series3Set);

      XYSeriesCollection xySeriesCollection = GraphingUtility.createXYSeriesCollection(data);

      ChartPanel chartPanel = GraphingUtility.createGraph("BIN Noise vs. Flight Path Angle", "Flight Path Angle (degrees)", "BIN Noise", xySeriesCollection);
      ChartPanel chartPanel2 = GraphingUtility.createGraph("BIN Noise vs. Flight Path Angle", "Flight Path Angle (degrees)", "BIN Noise", xySeriesCollection);

      JPanel jPanel = new JPanel();
      jPanel.setLayout(new GridBagLayout());
      GridBagConstraints gbc = new GridBagConstraints();
      gbc.fill = GridBagConstraints.BOTH;
      gbc.weightx = 1.0;
      gbc.weighty = 1.0;

      jPanel.add(chartPanel, gbc);
      gbc.gridy++;
      GraphingUtility.increaseFontSize(chartPanel2.getChart(), 6);
      jPanel.add(chartPanel2, gbc);

      JFrame jFrame = new JFrame("Test Graph");
      jFrame.getContentPane().add(jPanel);
      jFrame.pack();
      jFrame.setVisible(true);
   }
}
