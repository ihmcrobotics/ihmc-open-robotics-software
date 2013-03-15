package us.ihmc.imageProcessing.segmentation;

import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;
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

   XYSeriesCollection dataset = new XYSeriesCollection();

   public VisualizeScatter() {
      super( new BorderLayout() );

      setPreferredSize(new java.awt.Dimension(250, 250));
      setMinimumSize(getPreferredSize());
      setMaximumSize(getPreferredSize());

      JFreeChart chart = ChartFactory.createScatterPlot(
              null,
              "H", "S",
              dataset,
              PlotOrientation.VERTICAL,
              false,false,false);

      NumberAxis domainAxis = (NumberAxis) chart.getXYPlot().getDomainAxis();
      domainAxis.setRange(0,2*Math.PI);
      NumberAxis rangeAxis = (NumberAxis) chart.getXYPlot().getRangeAxis();
      rangeAxis.setRange(0,1);

      ChartPanel chartPanel = new ChartPanel(chart);

      add(chartPanel, BorderLayout.CENTER);
   }


   public void update( MultiSpectral<ImageFloat32> color , ImageUInt8 binary ) {
      dataset.removeAllSeries();

      XYSeries series = new XYSeries("Hue-Sat");

      ImageFloat32 H = color.getBand(0);
      ImageFloat32 S = color.getBand(1);

      for( int y = 0; y < binary.height; y++ ) {
         int index = binary.startIndex + y*binary.stride;

         for( int x = 0; x < binary.width; x++ ) {
           if( binary.data[index++] == 1 ) {
              float h = H.unsafe_get(x,y);
              float v = S.unsafe_get(x,y);

              series.add(h,v);
           }
         }
      }

      dataset.addSeries(series);

      repaint();
   }

}
