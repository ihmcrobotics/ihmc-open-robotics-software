package us.ihmc.plotting.jfree;

public class JFreePlotterDemo
{
   public static void main(String[] args)
   {
      jFreePlotter plot = new jFreePlotter("");
      double[] xdata = new double[] {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
      double[] ydata1 = new double[] {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
      double[] ydata2 = new double[] {0, 1, 1, 1, 2, 2, 6, 6, 9, 9};
      String[] xLabels = new String[] {"x1-label", "x2-label"};
      String[] yLabels = new String[] {"y1-label", "y2-label"};

      plot.setXData(xdata, "x-label");
      plot.setYData(ydata1, "y1-label");
      plot.setXAxisLogarithmic(false);
      plot.setLocation(50, 0);
      plot.plotXYLineChart();

      plot.setXData(xdata, "x-label", 2);
      plot.setYData(ydata2, "y2-label", 2);
      plot.setLocation(600, 0);
      plot.plotXYLineChart(2);

      plot.setXData(xdata, "x-label", 1);
      plot.setYData(ydata1, "y1-label", 1);
      plot.setXData(xdata, "x-label", 2);
      plot.setYData(ydata2, "y2-label", 2);
      plot.setLocation(50, 300);
      plot.plotXYLineChart();

      plot.setXLabel(xLabels);
      plot.setYLabel(yLabels);
      plot.setLocation(600, 300);
      plot.plotCombinedXYLineChart();
   }
}
