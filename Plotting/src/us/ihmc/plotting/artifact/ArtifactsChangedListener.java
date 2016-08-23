package us.ihmc.plotting.artifact;

import java.util.ArrayList;

public interface ArtifactsChangedListener
{
   public void artifactsChanged(ArrayList<Artifact> newArtifacts);
}
