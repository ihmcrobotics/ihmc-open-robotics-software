package us.ihmc.exampleSimulations.singgleLeggedRobot;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.ApplicationFrame;

public class TestPlot extends ApplicationFrame
{

   /**
    * A demonstration application showing an XY series containing a null value.
    *
    * @param title the frame title.
    */
   public TestPlot(final String title, MatrixForML plotData)
   {
      super(title);
      final XYSeries series = new XYSeries("Data Plot");
      
      for(int i=0; i<plotData.getCol(); i++)
      {
         series.add(plotData.getDoubleValue(0, i),plotData.getDoubleValue(1, i));
      }
      
      final XYSeriesCollection data = new XYSeriesCollection(series);
      final JFreeChart chart = ChartFactory.createXYLineChart(title, "X", "Y", data, PlotOrientation.VERTICAL, true, true, false);

      final ChartPanel chartPanel = new ChartPanel(chart);
      chartPanel.setPreferredSize(new java.awt.Dimension(500, 270));
      setContentPane(chartPanel);

   }

   //****************************************************************************
   //* JFREECHART DEVELOPER GUIDE                                               *
   //* The JFreeChart Developer Guide, written by David Gilbert, is available   *
   //* to purchase from Object Refinery Limited:                                *
   //*                                                                          *
   //* http://www.object-refinery.com/jfreechart/guide.html                     *
   //*                                                                          *
   //* Sales are used to provide funding for the JFreeChart project - please    * 
   //* support us so that we can continue developing free software.             *
   //****************************************************************************

   /**
    * Starting point for the demonstration application.
    *
    * @param args ignored.
    */
   
//      final TestPlot demo = new TestPlot("XY Series Demo");
//      demo.pack();
//      RefineryUtilities.centerFrameOnScreen(demo);
//      demo.setVisible(true);

}