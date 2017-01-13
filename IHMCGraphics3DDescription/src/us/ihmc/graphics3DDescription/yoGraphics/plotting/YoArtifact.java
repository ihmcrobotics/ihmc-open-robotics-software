package us.ihmc.graphics3DDescription.yoGraphics.plotting;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.graphics3DDescription.appearance.AppearanceDefinition;
import us.ihmc.graphics3DDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphics3DDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphics3DDescription.plotting.artifact.Artifact;
import us.ihmc.graphics3DDescription.yoGraphics.RemoteYoGraphic;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public abstract class YoArtifact extends Artifact implements RemoteYoGraphic
{
   private final YoVariable<?>[] variableArray;
   private final double[] constants;
   private final AppearanceDefinition appearance;
   
   private final ArrayList<double[]> historicalData = new ArrayList<double[]>();
   
   public YoArtifact(String name, double[] constants, Color color, YoVariable<?>... variableArray)
   {
      super(name);
      
      this.variableArray = variableArray;
      this.constants = constants;
      this.appearance = new YoAppearanceRGBColor(color, 0.0);
      this.color = color;
   }
   
   public abstract void drawHistoryEntry(Graphics2DAdapter graphics, double[] entry);
   
   @Override
   public YoVariable<?>[] getVariables()
   {
      return variableArray;
   }
   
   @Override
   public final double[] getConstants()
   {
      return constants;
   }
   
   @Override
   public final AppearanceDefinition getAppearance()
   {
      return appearance;
   }
   
   @Override
   public final String getName()
   {
      return getID();
   }
   
   @Override
   public final void takeHistorySnapshot()
   {
      if (getRecordHistory())
      {
         synchronized (historicalData)
         {
            double[] values = new double[variableArray.length];
            for (int i = 0; i < variableArray.length; i++)
            {
               values[i] = variableArray[i].getValueAsDouble();
            }
            historicalData.add(values);
         }
      }
   }
   
   @Override
   public final void drawHistory(Graphics2DAdapter graphics)
   {
      synchronized (historicalData)
      {
         for (double[] data : historicalData)
         {
            drawHistoryEntry(graphics, data);
         }
      }
   }
}
