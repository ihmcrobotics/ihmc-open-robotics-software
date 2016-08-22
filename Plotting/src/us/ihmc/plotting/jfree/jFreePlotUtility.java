package us.ihmc.plotting.jfree;

import java.awt.Color;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
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
 * @author Seyed Hossein Tamaddoni
 */
public class jFreePlotUtility extends JFrame
{
   private static final long serialVersionUID = 2986192525063918143L;

   public jFreePlotUtility(String figureName)
   {
      super(figureName);
   }

   public void createXYLineChart(double[] xDATA, double[] yDATA, String TITLE, String XLABEL, String YLABEL, int WIDTH, int HEIGHT, boolean ISXAXISLOGARITHMIC,
                                 boolean ISYAXISLOGARITHMIC)
   {
      XYDataset dataset = createDataset(xDATA, yDATA);

      JFreeChart chart = ChartFactory.createXYLineChart(TITLE, XLABEL, YLABEL, dataset, PlotOrientation.VERTICAL, false, false, false);
      chart.setBackgroundPaint(Color.lightGray);
      XYPlot plot = (XYPlot) chart.getPlot();

      NumberAxis rangeAxis, domainAxis;
      if (ISXAXISLOGARITHMIC)
         domainAxis = new LogarithmicAxis(XLABEL);
      else
         domainAxis = new NumberAxis(XLABEL);

      if (ISYAXISLOGARITHMIC)
         rangeAxis = new LogarithmicAxis(YLABEL);
      else
         rangeAxis = new NumberAxis(YLABEL);

      plot.setDomainAxis(domainAxis);
      plot.setRangeAxis(rangeAxis);
      plot.setBackgroundPaint(Color.white);
      plot.setDomainGridlinesVisible(true);
      plot.setDomainGridlinePaint(Color.blue);

      ChartPanel chartPanel = new ChartPanel(chart);
      chartPanel.setPreferredSize(new java.awt.Dimension(WIDTH, HEIGHT));
      this.setContentPane(chartPanel);
   }

   public void createXYLineChart(double[][] xDATA, double[][] yDATA, String TITLE, String XLABEL, String YLABEL, int WIDTH, int HEIGHT,
                                 boolean ISXAXISLOGARITHMIC, boolean ISYAXISLOGARITHMIC)
   {
      XYDataset dataset = createDataset(xDATA, yDATA);

      JFreeChart chart = ChartFactory.createXYLineChart(TITLE, XLABEL, YLABEL, dataset, PlotOrientation.VERTICAL, false, false, false);
      chart.setBackgroundPaint(Color.lightGray);
      XYPlot plot = (XYPlot) chart.getPlot();

      NumberAxis rangeAxis, domainAxis;
      if (ISXAXISLOGARITHMIC)
         domainAxis = new LogarithmicAxis(XLABEL);
      else
         domainAxis = new NumberAxis(XLABEL);

      if (ISYAXISLOGARITHMIC)
         rangeAxis = new LogarithmicAxis(YLABEL);
      else
         rangeAxis = new NumberAxis(YLABEL);

      plot.setDomainAxis(domainAxis);
      plot.setRangeAxis(rangeAxis);
      plot.setBackgroundPaint(Color.white);
      plot.setDomainGridlinesVisible(true);
      plot.setDomainGridlinePaint(Color.blue);

      ChartPanel chartPanel = new ChartPanel(chart);
      chartPanel.setPreferredSize(new java.awt.Dimension(WIDTH, HEIGHT));
      this.setContentPane(chartPanel);
   }

   public void createCombinedChart(double[][] xDATA, double[][] yDATA, String TITLE, String[] XLABEL, String[] YLABEL, int WIDTH, int HEIGHT,
                                   boolean ISXAXISLOGARITHMIC, boolean ISYAXISLOGARITHMIC)
   {
      int numberOfPlots = xDATA.length;
      XYDataset[] dataset = new XYDataset[numberOfPlots];
      XYItemRenderer[] renderer = new XYItemRenderer[numberOfPlots];
      NumberAxis[] rangeAxis = new NumberAxis[numberOfPlots];
      NumberAxis[] valueAxis = new NumberAxis[numberOfPlots];
      XYPlot[] subplot = new XYPlot[numberOfPlots];

      for (int i = 0; i < numberOfPlots; i++)
      {
         dataset[i] = createDataset(xDATA[i], yDATA[i]);

         if (ISXAXISLOGARITHMIC)
            valueAxis[i] = new LogarithmicAxis(XLABEL[i]);
         else
            valueAxis[i] = new NumberAxis(XLABEL[i]);

         if (ISYAXISLOGARITHMIC)
            rangeAxis[i] = new LogarithmicAxis(YLABEL[i]);
         else
            rangeAxis[i] = new NumberAxis(YLABEL[i]);

         rangeAxis[i].setAutoRangeIncludesZero(false);
         subplot[i] = new XYPlot(dataset[i], null, rangeAxis[i], renderer[i]);
         subplot[i].setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
         renderer[i] = new StandardXYItemRenderer();
         renderer[i].setSeriesVisibleInLegend(0, false);
      }

      CombinedDomainXYPlot plot = new CombinedDomainXYPlot(valueAxis[0]);
      plot.setGap(10.0);

      for (int i = 0; i < numberOfPlots; i++)
      {
         plot.add(subplot[i], 1);
      }

      plot.setOrientation(PlotOrientation.VERTICAL);
      JFreeChart chart = new JFreeChart(TITLE, JFreeChart.DEFAULT_TITLE_FONT, plot, true);
      ChartPanel chartPanel = new ChartPanel(chart);
      chartPanel.setPreferredSize(new java.awt.Dimension(WIDTH, HEIGHT));
      setContentPane(chartPanel);
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

   private XYDataset createDataset(double[][] xdata, double[][] ydata)
   {
      XYSeries series = new XYSeries("data series", false);
      XYSeriesCollection dataset = new XYSeriesCollection();
      for (int i = 0; i < xdata.length; i++)
      {
         for (int j = 0; j < xdata[i].length; j++)
         {
            series.add(xdata[i][j], ydata[i][j]);
         }

         dataset.addSeries(series);
      }

      return dataset;
   }
}
