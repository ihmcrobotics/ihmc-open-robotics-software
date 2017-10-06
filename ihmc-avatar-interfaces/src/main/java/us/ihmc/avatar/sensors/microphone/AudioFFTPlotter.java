package us.ihmc.avatar.sensors.microphone;

import javax.swing.JFrame;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.AxisLocation;
import org.jfree.chart.axis.LogarithmicAxis;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class AudioFFTPlotter extends JFrame
{

   private static final long serialVersionUID = 8401116583400863812L;

   public AudioFFTPlotter(double[] freqData, double[] magData, String title, String freqUnits, String magnitudeUnits)
   {
      super(title);
      double[] freq = freqData;
      double[] magnitude = magData;

      if (freq[0] < 1e-7)
      {
         freq = throwFirstPointAway(freq);
         magnitude = throwFirstPointAway(magnitude);
      }

      XYDataset magDataset = createDataset(freq, magnitude);
      JFreeChart chart = createChart(title, magDataset, freqUnits, magnitudeUnits);
      ChartPanel chartPanel = new ChartPanel(chart);
      chartPanel.setPreferredSize(new java.awt.Dimension(500, 270));
      this.setContentPane(chartPanel);
   }

   public void packAndDisplayFrame(int xScreen, int yScreen)
   {
      pack();
      setLocation(xScreen, yScreen);
      setVisible(true);
   }


   private double[] throwFirstPointAway(double[] data)
   {
      double[] ret = new double[data.length - 1];

      for (int i = 1; i < data.length; i++)
      {
         ret[i - 1] = data[i];
      }
      return ret;
   }

   private XYDataset createDataset(double[] xdata, double[] ydata)
   {
      XYSeries series = new XYSeries("data series", false);
      XYSeriesCollection dataset = new XYSeriesCollection();

      for (int i = 0; i < xdata.length; i++)
      {
         series.add(xdata[i], ydata[i]);
      }

      dataset.addSeries(series);

      return dataset;
   }

   private JFreeChart createChart(String title, XYDataset magDataset, String freqUnits, String magnitudeUnits)
   {
      XYItemRenderer renderer1 = new StandardXYItemRenderer();
      NumberAxis rangeAxis1 = new NumberAxis("Magnitude " + magnitudeUnits);
      XYPlot subplot1 = new XYPlot(magDataset, new LogarithmicAxis("Frequency " + freqUnits), rangeAxis1, renderer1);
      subplot1.setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
      renderer1.setSeriesVisibleInLegend(0, false);
      return new JFreeChart(title, JFreeChart.DEFAULT_TITLE_FONT, subplot1, true);
   }
}