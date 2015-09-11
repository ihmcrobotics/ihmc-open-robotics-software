package us.ihmc.yoUtilities.graphics.plotting;

import java.util.ArrayList;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Plotter;

public class ArtifactList
{
   private String label;
   private ArrayList<Artifact> artifacts;

   public ArtifactList(String label, ArrayList<Artifact> artifacts)
   {
      this.label = label;
      this.artifacts = artifacts;
   }

   public ArtifactList(String label, Artifact[] artifactArray)
   {
      this.label = label;

      ArrayList<Artifact> artifacts = new ArrayList<Artifact>(artifactArray.length);

      for (Artifact artifact : artifactArray)
      {
         artifacts.add(artifact);
      }

      this.artifacts = artifacts;
   }

   public ArtifactList(String label, Artifact artifact)
   {
      this.label = label;

      ArrayList<Artifact> artifacts = new ArrayList<Artifact>(1);

      artifacts.add(artifact);

      this.artifacts = artifacts;
   }

   public ArtifactList(String label)
   {
      this.label = label;
      this.artifacts = new ArrayList<Artifact>();
   }

   public String getLabel()
   {
      return label;
   }

   public ArrayList<Artifact> getArtifacts()
   {
      return artifacts;
   }

   public void add(Artifact artifact)
   {
      this.artifacts.add(artifact);
   }

   public void addAll(ArrayList<Artifact> artifacts)
   {
      this.artifacts.addAll(artifacts);
   }

   public void addArtifactsToPlotter(Plotter plotter)
   {
      for (Artifact artifact : artifacts)
      {
         plotter.addArtifact(artifact);
      }
   }
}
