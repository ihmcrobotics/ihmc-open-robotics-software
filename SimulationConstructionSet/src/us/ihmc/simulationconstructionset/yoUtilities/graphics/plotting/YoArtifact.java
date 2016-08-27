package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.Color;

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
   public double[] getConstants()
   {
      return constants;
   }
   
   @Override
   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }
}
