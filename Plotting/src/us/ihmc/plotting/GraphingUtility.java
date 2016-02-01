package us.ihmc.plotting;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Point2d;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.labels.StandardXYToolTipGenerator;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.ValueMarker;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

/**
 * User: mjohnson
 * Date: 6/6/12
 */
public class GraphingUtility
{
   public static ChartPanel createGraph(String title, String xAxisLabel, String yAxisLabel, XYDataset dataset)
   {
      return createGraph(title, xAxisLabel, yAxisLabel, dataset, true, false, true);
   }

   public static ChartPanel createGraph(String title, String xAxisLabel, String yAxisLabel, XYDataset dataset, boolean showLegend)
   {
      return createGraph(title, xAxisLabel, yAxisLabel, dataset, true, false, showLegend);
   }

   public static ChartPanel createGraph(String title, String xAxisLabel, String yAxisLabel, XYDataset dataset, boolean showSeriesShape, boolean independentYaxes)
   {
      return createGraph(title, xAxisLabel, yAxisLabel, dataset, showSeriesShape, independentYaxes, true);
   }

   public static ChartPanel createGraph(String title, String xAxisLabel, String yAxisLabel, XYDataset dataset, boolean showSeriesShape, boolean independentYaxes, boolean showLegend)
   {
      // create the chart
      final JFreeChart chart = ChartFactory.createXYLineChart(title,    // chart title
         xAxisLabel,    // x axis label
         yAxisLabel,    // y axis label
         dataset,    // data
         PlotOrientation.VERTICAL,
         showLegend, // include legend
         true,    // tooltips
         false    // urls
            );

      // NOW DO SOME OPTIONAL CUSTOMISATION OF THE CHART...
      chart.setBackgroundPaint(Color.white);

      // final StandardLegend legend = (StandardLegend) chart.getLegend();
      // legend.setDisplaySeriesShapes(true);

      // get a reference to the plot for further customisation...
      final XYPlot plot = chart.getXYPlot();
      plot.setBackgroundPaint(Color.white);

      // plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
      plot.setDomainGridlinePaint(Color.lightGray);
      plot.setRangeGridlinePaint(Color.lightGray);
      
      
      final XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
       renderer.setBaseToolTipGenerator(new StandardXYToolTipGenerator());

      // renderer.setSeriesLinesVisible(0, false);
      // renderer.setSeriesShapesVisible(1, false);

      if (!showSeriesShape)
      {
         for (int i = 0; i < dataset.getSeriesCount(); i++)
         {
            renderer.setSeriesShapesVisible(i, false);
         }
      }


      plot.setRenderer(renderer);

      // change the auto tick unit selection to integer units only...
      final NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
      rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());

      if (independentYaxes && plot.getSeriesCount() > 1)
      {  
         for(int i=0; i<plot.getSeriesCount(); i++)
         {
            NumberAxis axis = new NumberAxis("Additional Axes " + i);
            axis.setAutoRange(true);
            plot.setRangeAxis(1, axis);
            plot.mapDatasetToRangeAxis(1, 1);            
         }
         
      }
      
//      
//      plot.setRangeAxis(1,valueAxis1);
      
      // plot.getRangeAxis().setRange(90.0, 100.0);

      
      ChartPanel chartPanel = new ChartPanel(chart);

    
      // chartPanel.setPreferredSize(new java.awt.Dimension(600, 400));
      return chartPanel;
   }
   
   /**
    * 
    * @param chartPanel
    * @param dataset
    * @param yAxisLabel
    * @param showSeriesShape
    * @param independentYaxis
    * @param yAxisMin Set this to NaN to use autoscaling
    * @param yAxisMax
    */
   public static void addDataSetToXYPlot(ChartPanel chartPanel, XYDataset dataset, String yAxisLabel, boolean showSeriesShape, boolean independentYaxis, double yAxisMin, double yAxisMax)
   {
      XYPlot plot = chartPanel.getChart().getXYPlot();
      
      int seriesCount = plot.getSeriesCount();
      if (independentYaxis)
      {
         NumberAxis axis = new NumberAxis(yAxisLabel);
         if(Double.isNaN(yAxisMin))
            axis.setAutoRange(true);
         else
            axis.setRange(yAxisMin, yAxisMax);
         
         plot.setRangeAxis(seriesCount, axis);
         
         plot.mapDatasetToRangeAxis(seriesCount, seriesCount);   
      }

      plot.setDataset(seriesCount, dataset);

//       XYLineAndShapeRenderer xyLineAndShapeRenderer = new XYLineAndShapeRenderer();
       XYLineAndShapeRenderer xyLineAndShapeRenderer = (XYLineAndShapeRenderer) plot.getRenderer();           // get the base xyrenderer
//      xyLineAndShapeRenderer.setSeriesPaint(seriesCount, plot.getDrawingSupplier().getNextPaint());
      plot.setRenderer(seriesCount, xyLineAndShapeRenderer);
       xyLineAndShapeRenderer.setSeriesToolTipGenerator(seriesCount, new StandardXYToolTipGenerator());
      if (!showSeriesShape)
      {
//         xyLineAndShapeRenderer = (XYLineAndShapeRenderer) plot.getRenderer();
         xyLineAndShapeRenderer.setSeriesShapesVisible(0, false);   
      }      
   }

   
   public static void addVerticalMarkerToXYPlot(ChartPanel chartPanel, double xValue)
   {
      ValueMarker marker = new ValueMarker(xValue);
      marker.setPaint(Color.BLACK);
      
      XYPlot plot = (XYPlot) chartPanel.getChart().getPlot();
      plot.addDomainMarker(marker);
   }


   public static void increaseFontSize(JFreeChart chart, int amount)
   {
      XYPlot plot = chart.getXYPlot();

      Font font = plot.getDomainAxis().getTickLabelFont();
      int size = font.getSize();
      size += amount;
      Font bigger = new Font(font.getName(), font.getStyle(), size);
      ValueAxis valueAxis = plot.getDomainAxis();
      valueAxis.setTickLabelFont(bigger);

      font = plot.getDomainAxis().getLabelFont();
      size = font.getSize();
      size += amount;
      bigger = new Font(font.getName(), font.getStyle(), size);
      valueAxis.setLabelFont(bigger);

      plot.setDomainAxis(valueAxis);

      font = plot.getRangeAxis().getTickLabelFont();
      size = font.getSize();
      size += amount;
      bigger = new Font(font.getName(), font.getStyle(), size);
      valueAxis = plot.getRangeAxis();
      valueAxis.setTickLabelFont(bigger);

      font = plot.getRangeAxis().getLabelFont();
      size = font.getSize();
      size += amount;
      bigger = new Font(font.getName(), font.getStyle(), size);
      valueAxis.setLabelFont(bigger);

      plot.setRangeAxis(valueAxis);

      font = chart.getTitle().getFont();
      size = font.getSize();
      size += amount;
      bigger = new Font(font.getName(), font.getStyle(), size);
      chart.getTitle().setFont(bigger);

      plot.setRangeAxis(valueAxis);

       if ( chart.getLegend() != null )
       {
           font = chart.getLegend().getItemFont();
           size = font.getSize();
           size += amount;
           bigger = new Font(font.getName(), font.getStyle(), size);
           chart.getLegend().setItemFont(bigger);
       }
   }

    public static ChartPanel createGraph(String title, String xAxisLabel, String yAxisLabel, XYDataset dataset, double minRange, double maxRange)
    {
       return createGraph(title,xAxisLabel,yAxisLabel,dataset,minRange,maxRange,true);
    }

    public static ChartPanel createGraph(String title, String xAxisLabel, String yAxisLabel, XYDataset dataset, double minRange, double maxRange, boolean showLegend)
   {
      // create the chart
      final JFreeChart chart = ChartFactory.createXYLineChart(title,    // chart title
         xAxisLabel,    // x axis label
         yAxisLabel,    // y axis label
         dataset,    // data
         PlotOrientation.VERTICAL,
         showLegend,    // include legend
         true,    // tooltips
         false    // urls
            );

      // NOW DO SOME OPTIONAL CUSTOMISATION OF THE CHART...
      chart.setBackgroundPaint(Color.white);

      // final StandardLegend legend = (StandardLegend) chart.getLegend();
      // legend.setDisplaySeriesShapes(true);

      // get a reference to the plot for further customisation...
      final XYPlot plot = chart.getXYPlot();
      plot.setBackgroundPaint(Color.white);

      // plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
      plot.setDomainGridlinePaint(Color.lightGray);
      plot.setRangeGridlinePaint(Color.lightGray);

      final XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
       renderer.setBaseToolTipGenerator(new StandardXYToolTipGenerator());

      // renderer.setSeriesLinesVisible(0, false);
      // renderer.setSeriesShapesVisible(1, false);
      plot.setRenderer(renderer);

      // change the auto tick unit selection to integer units only...
      final NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
      rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());

      plot.getRangeAxis().setRange(minRange, maxRange);

      ChartPanel chartPanel = new ChartPanel(chart);

      // chartPanel.setPreferredSize(new java.awt.Dimension(600, 400));
      return chartPanel;
   }

   public static void displayGraph(String title, String xAxisLabel, String yAxisLabel, XYDataset dataset)
   {
      ChartPanel chartPanel = createGraph(title, xAxisLabel, yAxisLabel, dataset);
      JFrame jFrame = new JFrame("Test Graph");
      jFrame.getContentPane().add(chartPanel);
      jFrame.pack();
      jFrame.setVisible(true);
   }

   public static void displayGraph(String title, String xAxisLabel, String yAxisLabel, XYDataset dataset, double minRange, double maxRange)
   {
      ChartPanel chartPanel = createGraph(title, xAxisLabel, yAxisLabel, dataset, minRange, maxRange);
      JFrame jFrame = new JFrame("Test Graph");
      jFrame.getContentPane().add(chartPanel);
      jFrame.pack();
      jFrame.setVisible(true);
   }

   public static XYSeries createXYSeries(String seriesName, ArrayList<Point2d> data)
   {
      final XYSeries series1 = new XYSeries(seriesName);
      for (int i = 0; i < data.size(); i++)
      {
         Point2d point2d = data.get(i);
         series1.add(point2d.getX(), point2d.getY());
      }

      return series1;
   }

   public static XYSeriesCollection createXYSeriesCollection(ArrayList<XYSeries> dataSets)
   {
      final XYSeriesCollection dataSet = new XYSeriesCollection();
      for (int i = 0; i < dataSets.size(); i++)
      {
         dataSet.addSeries(dataSets.get(i));
      }

      return dataSet;
   }

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
