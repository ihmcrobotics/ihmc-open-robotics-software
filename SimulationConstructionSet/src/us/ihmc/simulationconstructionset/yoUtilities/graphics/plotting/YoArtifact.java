package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.RemoteYoGraphic;

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
   }
   
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
}
