package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.util.ArrayList;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Plotter;

public class ArtifactList
{
   private String label;
   private ArrayList<Artifact> artifacts;

   public ArtifactList(String label, ArrayList<Artifact> artifacts)
   {
      // TODO: should a defensive copy be made of artifacts?
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

   public void setVisible(boolean visible)
   {
      if (artifacts != null)
      {
         int numberOfElements = artifacts.size();

         for (int i = 0; i < numberOfElements; i++)
         {
            Artifact artifact = artifacts.get(i);
            artifact.setVisible(visible);
         }
      }
   }

   public void hideArtifacts()
   {
      if (artifacts != null)
      {
         int numberOfElements = artifacts.size();

         for (int i = 0; i < numberOfElements; i++)
         {
            Artifact artifact = artifacts.get(i);
            artifact.setVisible(false);
         }
      }
   }
}
