package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import us.ihmc.plotting.artifact.Artifact;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.RemoteYoGraphic;

public abstract class YoArtifact extends Artifact implements RemoteYoGraphic
{
   public YoArtifact(String name)
   {
      super(name);
   }
}
