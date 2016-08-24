package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import us.ihmc.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.RemoteYoGraphic;

public abstract class YoArtifact extends Artifact implements RemoteYoGraphic
{
   protected final YoVariable<?>[] variableArray; 
   
   public YoArtifact(String name, YoVariable<?>... variableArray)
   {
      super(name);
      
      this.variableArray = variableArray;
   }
   
   @Override
   public YoVariable<?>[] getVariables()
   {
      return variableArray;
   }
}
