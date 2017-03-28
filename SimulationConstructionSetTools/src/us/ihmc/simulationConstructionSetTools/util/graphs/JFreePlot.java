package us.ihmc.simulationConstructionSetTools.util.graphs;

import java.awt.BasicStroke;
import java.awt.Color;

import org.jfree.data.xy.XYSeries;

import us.ihmc.simulationconstructionset.DataBufferEntry;

public class JFreePlot extends XYSeries
{
   private static final long serialVersionUID = -5258162352646570327L;
   public enum PlotTypes {Dash, Solid, Dot}

   private PlotTypes type = PlotTypes.Solid;

   private Color color = Color.BLACK;
   private String name;

   private final BasicStroke solid = new BasicStroke(2.0f);
   private final BasicStroke doted = new BasicStroke(2.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 1.0f, new float[] {1.0f, 6.0f}, 0.0f);
   private final BasicStroke dashed = new BasicStroke(2.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 1.0f, new float[] {6.0f, 6.0f}, 0.0f);

   private boolean isScatterPlot = false;

   protected JFreePlot(String name)
   {
      super(name);
   }
   
   public JFreePlot(DataBufferEntry xPlot, DataBufferEntry yPlot)
   {
      this(xPlot.getData(), yPlot.getData());
   }

   public JFreePlot(double[] xPlot, double[] yPlot)
   {
      this("plot", xPlot, yPlot);
   }

   public JFreePlot(DataBufferEntry xPlot, DataBufferEntry yPlot, boolean autoSort, boolean allowDuplicateXValues)
   {
      this(xPlot.getData(), yPlot.getData(), autoSort, allowDuplicateXValues);
   }

   public JFreePlot(double[] xPlot, double[] yPlot, boolean autoSort, boolean allowDuplicateXValues)
   {
      this("plot", xPlot, yPlot, autoSort, allowDuplicateXValues);
   }

   public JFreePlot(String name, DataBufferEntry xPlot, DataBufferEntry yPlot)
   {
      this(name, xPlot.getData(), yPlot.getData());
   }

   public JFreePlot(String name, double[] xPlot, double[] yPlot)
   {
      this(name, xPlot, yPlot, true, false);
   }

   public JFreePlot(String name, DataBufferEntry xPlot, DataBufferEntry yPlot, boolean autoSort, boolean allowDuplicateXValues)
   {
      this(name, xPlot.getData(), yPlot.getData(), autoSort, allowDuplicateXValues);
   }

   public JFreePlot(String name, double[] xPlot, double[] yPlot, boolean autoSort, boolean allowDuplicateXValues)
   {
      super(name, autoSort, allowDuplicateXValues);
      createXYSeries(xPlot, yPlot);
   }

   protected void createXYSeries(double[] xData, double[] yData)
   {
      if (xData.length != yData.length)
      {
         throw new RuntimeException("xData.length != yData.length");
      }

      int numberDataPoints = xData.length;

      try
      {
         for (int i = 1; i < numberDataPoints; i++)
         {
            double xValue = xData[i];
            double yValue = yData[i];

            add(xValue, yValue);
         }
      }
      catch (Exception e)
      {
         System.err.println("Duplicate x value found, Trimming Graph at this location");
      }
   }

   public void setType(PlotTypes type)
   {
      this.type = type;
   }

   protected BasicStroke getBasicStroke()
   {
      BasicStroke returnType = null;
      switch (type)
      {
         case Dash :
            returnType = dashed;

            break;

         case Dot :
            returnType = doted;

            break;

         default :
            returnType = solid;

            break;
      }

      return returnType;
   }

   public boolean isScatterPlot()
   {
      return isScatterPlot;
   }

   public void setIsScatterPlot(boolean isScatterPlot)
   {
      this.isScatterPlot = isScatterPlot;
   }

   public Color getColor()
   {
      return color;
   }

   public void setColor(Color color)
   {
      this.color = color;
   }

   public String getName()
   {
      return name;
   }
}
