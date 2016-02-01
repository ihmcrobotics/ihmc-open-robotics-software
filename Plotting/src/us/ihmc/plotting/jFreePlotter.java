package us.ihmc.plotting;


/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2008</p>
 *
 * <p>Company: </p>
 *
 * @author Seyed Hossein Tamaddoni
 * @version 1.0
 */
public class jFreePlotter
{
   private String TITLE = "jFree Chart";
   private String[] XLABEL = new String[]
   {
      "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10"
   };
   private String[] YLABEL = new String[]
   {
      "y1", "y2", "y3", "y4", "y5", "y6", "y7", "y8", "y9", "y10"
   };
   private int WIDTH = 500;
   private int HEIGHT = 270;
   private double[][] XDATA = new double[10][];
   private double[][] YDATA = new double[10][];
   private String FIGURENAME = "";
   private int NUMBEROFDATA = 1;
   private boolean ISXAXISLOGARITHMIC = false;
   private boolean ISYAXISLOGARITHMIC = false;
   private int XLOCATION = 0;
   private int YLOCATION = 0;

   public jFreePlotter(String figureName)
   {
      FIGURENAME = figureName;
   }

   public void setLocation(int xLocation, int yLocation)
   {
      XLOCATION = xLocation;
      YLOCATION = yLocation;
   }

   public void setXAxisLogarithmic(boolean isXAxisLogarithmic)
   {
      ISXAXISLOGARITHMIC = isXAxisLogarithmic;
   }

   public void setYAxisLogarithmic(boolean isYAxisLogarithmic)
   {
      ISYAXISLOGARITHMIC = isYAxisLogarithmic;
   }

   public void setWidth(int width)
   {
      WIDTH = width;
   }

   public void setHeight(int height)
   {
      HEIGHT = height;
   }

   public void setTitle(String title)
   {
      TITLE = title;
   }

   public void setXLabel(String[] xLabel)
   {
      XLABEL = xLabel;
   }

   public void setYLabel(String[] yLabel)
   {
      YLABEL = yLabel;
   }

   public void setLabels(String title, String[] xLabel, String[] yLabel)
   {
      setTitle(title);
      setXLabel(xLabel);
      setYLabel(yLabel);
   }

   public void setXLabel(String xLabel)
   {
      setXLabel(new String[] {xLabel});
   }

   public void setYLabel(String yLabel)
   {
      setYLabel(new String[] {yLabel});
   }

   public void setLabels(String title, String xLabel, String yLabel)
   {
      setTitle(title);
      setXLabel(new String[] {xLabel});
      setYLabel(new String[] {yLabel});
   }

   public void setXData(double[] xData)
   {
      setXData(xData, 1);
   }

   public void setXData(double[] xData, String[] xLabel)
   {
      setXData(xData, xLabel, 1);
   }

   public void setXData(double[] xData, String xLabel)
   {
      setXData(xData, new String[] {xLabel}, 1);
   }

   public void setYData(double[] yData)
   {
      setYData(yData, 1);
   }

   public void setYData(double[] yData, String[] yLabel)
   {
      setYData(yData, yLabel, 1);
   }

   public void setYData(double[] yData, String yLabel)
   {
      setYData(yData, new String[] {yLabel}, 1);
   }

   public void setXData(double[] xData, int dataNumber)
   {
      if (dataNumber > NUMBEROFDATA)
         NUMBEROFDATA = dataNumber;

      XDATA[dataNumber - 1] = new double[xData.length];

      for (int i = 0; i < xData.length; i++)
      {
         XDATA[dataNumber - 1][i] = xData[i];
      }
   }

   public void setXData(double[] xData, String[] xLabel, int dataNumber)
   {
      setXLabel(xLabel);
      setXData(xData, dataNumber);
   }

   public void setXData(double[] xData, String xLabel, int dataNumber)
   {
      setXData(xData, new String[] {xLabel}, dataNumber);
   }

   public void setYData(double[] yData, int dataNumber)
   {
      if (dataNumber > NUMBEROFDATA)
         NUMBEROFDATA = dataNumber;

      YDATA[dataNumber - 1] = new double[yData.length];

      for (int i = 0; i < yData.length; i++)
      {
         YDATA[dataNumber - 1][i] = yData[i];
      }
   }

   public void setYData(double[] yData, String[] yLabel, int dataNumber)
   {
      setYLabel(yLabel);
      setYData(yData, dataNumber);
   }

   public void setYData(double[] yData, String yLabel, int dataNumber)
   {
      setYData(yData, new String[] {yLabel}, dataNumber);
   }

   public void plotXYLineChart(int dataNumber)
   {
      jFreePlotUtility plot = new jFreePlotUtility(FIGURENAME);
      plot.createXYLineChart(XDATA[dataNumber - 1], YDATA[dataNumber - 1], TITLE, XLABEL[0], YLABEL[0], WIDTH, HEIGHT, ISXAXISLOGARITHMIC, ISYAXISLOGARITHMIC);
      packAndShow(plot);
   }

   public void plotXYLineChart()
   {
      double[][] xDATA = new double[NUMBEROFDATA][XDATA[0].length];
      double[][] yDATA = new double[NUMBEROFDATA][YDATA[0].length];

      for (int i = 0; i < NUMBEROFDATA; i++)
      {
         for (int j = 0; j < XDATA[i].length; j++)
         {
            xDATA[i][j] = XDATA[i][j];
            yDATA[i][j] = YDATA[i][j];
         }
      }

      jFreePlotUtility plot = new jFreePlotUtility(FIGURENAME);
      plot.createXYLineChart(xDATA, yDATA, TITLE, XLABEL[0], YLABEL[0], WIDTH, HEIGHT, ISXAXISLOGARITHMIC, ISYAXISLOGARITHMIC);
      packAndShow(plot);;
   }

   public void plotCombinedXYLineChart()
   {
      double[][] xDATA = new double[NUMBEROFDATA][XDATA[0].length];
      double[][] yDATA = new double[NUMBEROFDATA][YDATA[0].length];
      String[] xLABEL = new String[NUMBEROFDATA];
      String[] yLABEL = new String[NUMBEROFDATA];

      for (int i = 0; i < NUMBEROFDATA; i++)
      {
         for (int j = 0; j < XDATA[i].length; j++)
         {
            xDATA[i][j] = XDATA[i][j];
            yDATA[i][j] = YDATA[i][j];
         }

         xLABEL[i] = XLABEL[i];
         yLABEL[i] = YLABEL[i];
      }

      jFreePlotUtility plot = new jFreePlotUtility(FIGURENAME);
      plot.createCombinedChart(xDATA, yDATA, TITLE, xLABEL, yLABEL, WIDTH, HEIGHT, ISXAXISLOGARITHMIC, ISYAXISLOGARITHMIC);
      packAndShow(plot);
   }

   private void packAndShow(jFreePlotUtility plot)
   {
      plot.pack();
      plot.setLocation(XLOCATION, YLOCATION);
      plot.setVisible(true);
   }

   public static void main(String[] args)
   {
      jFreePlotter plot = new jFreePlotter("");
      double[] xdata = new double[]
      {
         0, 1, 2, 3, 4, 5, 6, 7, 8, 9
      };
      double[] ydata1 = new double[]
      {
         0, 1, 2, 3, 4, 5, 6, 7, 8, 9
      };
      double[] ydata2 = new double[]
      {
         0, 1, 1, 1, 2, 2, 6, 6, 9, 9
      };
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
