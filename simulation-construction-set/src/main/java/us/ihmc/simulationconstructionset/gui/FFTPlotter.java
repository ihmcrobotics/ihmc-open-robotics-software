package us.ihmc.simulationconstructionset.gui;

import javax.swing.JFrame;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.AxisLocation;
import org.jfree.chart.axis.LogarithmicAxis;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.CombinedDomainXYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;


/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * @author Seyed Hossein Tamaddoni
 * @see jFreeChart libraries should be added to the project library!
 */
public class FFTPlotter extends JFrame
{
// public static final String POSITION = "position";
// public static final String VELOCITY = "velocity";
// public static final String ACCELERATION = "acceleration";
// public static final String TIME = "time";

// public FFTPlotter(MotionGenerator profile, String xAxisTitle, String yAxisTitle)
// {
//     super(yAxisTitle + " -vs- " + xAxisTitle);
//     XYDataset dataset = createDataset(profile, xAxisTitle, yAxisTitle);
//     JFreeChart chart = createChart(yAxisTitle + "/" + xAxisTitle, dataset);
//     ChartPanel chartPanel = new ChartPanel(chart);
//     chartPanel.setPreferredSize(new java.awt.Dimension(500, 270));
//     setContentPane(chartPanel);
// }

   /**
    *
    */
   private static final long serialVersionUID = 8401116583400863812L;

   public FFTPlotter(double[][] freqMagnitudePhaseData, String title, String freqUnits, String magnitudeUnits, String phaseUnits)
   {
      super(title);
      double[] freq = freqMagnitudePhaseData[0];
      double[] magnitude = freqMagnitudePhaseData[1];
      double[] phase = freqMagnitudePhaseData[2];

      if (freq[0] < 1e-7)
      {
         freq = throwFirstPointAway(freq);
         magnitude = throwFirstPointAway(magnitude);
         phase = throwFirstPointAway(phase);
      }

      XYDataset magDataset = createDataset(freq, magnitude);
      XYDataset phaseDataset = createDataset(freq, phase);
      JFreeChart chart = createCombinedChart(title, magDataset, phaseDataset, freqUnits, magnitudeUnits, phaseUnits);
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

   private JFreeChart createCombinedChart(String title, XYDataset magDataset, XYDataset phaseDataset, String freqUnits, String magnitudeUnits,
           String phaseUnits)
   {
      XYItemRenderer renderer1 = new StandardXYItemRenderer();
      NumberAxis rangeAxis1 = new NumberAxis("Magnitude " + magnitudeUnits);
      XYPlot subplot1 = new XYPlot(magDataset, null, rangeAxis1, renderer1);
      subplot1.setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
      renderer1.setSeriesVisibleInLegend(0, false);

      XYItemRenderer renderer2 = new StandardXYItemRenderer();
      NumberAxis rangeAxis2 = new NumberAxis("Phase " + phaseUnits);
      rangeAxis2.setAutoRangeIncludesZero(false);
      XYPlot subplot2 = new XYPlot(phaseDataset, null, rangeAxis2, renderer2);
      subplot2.setRangeAxisLocation(AxisLocation.TOP_OR_LEFT);
      renderer2.setSeriesVisibleInLegend(0, false);

      CombinedDomainXYPlot plot = new CombinedDomainXYPlot(new LogarithmicAxis("Frequency " + freqUnits));
      plot.setGap(10.0);

      plot.add(subplot1, 1);
      plot.add(subplot2, 1);
      plot.setOrientation(PlotOrientation.VERTICAL);

      return new JFreeChart(title, JFreeChart.DEFAULT_TITLE_FONT, plot, true);
   }
}
