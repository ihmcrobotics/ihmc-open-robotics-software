package us.ihmc.imageProcessing.segmentation;

import boofcv.struct.image.GrayF32;
import boofcv.struct.image.InterleavedU8;
import boofcv.struct.image.Planar;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import javax.swing.*;
import java.awt.*;

/**
 * Displays selected pixel values in an X-Y scatter plot
 *
 * @author Peter Abeles
 */
public class VisualizeScatter extends JPanel {

   ChartInfo chartHS;
   ChartInfo chartVS;

   public VisualizeScatter( String name0, String name1 , String name2 , double max0 , double max1 , double max2 ) {
      GridLayout experimentLayout = new GridLayout(2,1);

      setLayout(experimentLayout);

      chartHS = createPlot(name0, name1, max0, max1);
      chartVS = createPlot(name2, name1, max2, max1);

      add(chartHS.panel);
      add(chartVS.panel);
   }

   private ChartInfo createPlot(String labelX, String labelY, double maxX, double maxY) {

      ChartInfo ret = new ChartInfo();

      JFreeChart chart = ChartFactory.createScatterPlot(
              null,
              labelX, labelY,
              ret.dataset,
              PlotOrientation.VERTICAL,
              false,false,false);

      NumberAxis domainAxis = (NumberAxis) chart.getXYPlot().getDomainAxis();
      domainAxis.setRange(0,maxX);
      NumberAxis rangeAxis = (NumberAxis) chart.getXYPlot().getRangeAxis();
      rangeAxis.setRange(0,maxY);

      ChartPanel panel = new ChartPanel(chart);

      panel.setPreferredSize(new Dimension(250, 250));
      panel.setMinimumSize(getPreferredSize());
      panel.setMaximumSize(getPreferredSize());

      ret.chart = chart;
      ret.panel = panel;

      return ret;
   }


   public void update(Planar<GrayF32> color , InterleavedU8 binary ) {
      // this is supposed to speed it up.  not sure if it does
      chartHS.chart.setNotify(false);
      chartVS.chart.setNotify(false);

      chartHS.dataset.removeAllSeries();
      chartVS.dataset.removeAllSeries();

      XYSeries seriesHS = new XYSeries("1");
      XYSeries seriesVS = new XYSeries("2");

      GrayF32 H = color.getBand(0);
      GrayF32 S = color.getBand(1);
      GrayF32 V = color.getBand(2);

      for( int y = 0; y < binary.height; y++ ) {
         int index = binary.startIndex + y*binary.stride;

         for( int x = 0; x < binary.width; x++ ) {
           if( binary.data[index++] == 1 ) {
              float h = H.unsafe_get(x,y);
              float s = S.unsafe_get(x,y);
              float v = V.unsafe_get(x,y);

              seriesHS.add(h, s);
              seriesVS.add(v, s);
           }
         }
      }

      chartHS.dataset.addSeries(seriesHS);
      chartVS.dataset.addSeries(seriesVS);

      chartHS.chart.setNotify(true);
      chartVS.chart.setNotify(true);

      repaint();
   }

   private static class ChartInfo
   {
      JFreeChart chart;
      XYSeriesCollection dataset = new XYSeriesCollection();
      ChartPanel panel;
   }

}
