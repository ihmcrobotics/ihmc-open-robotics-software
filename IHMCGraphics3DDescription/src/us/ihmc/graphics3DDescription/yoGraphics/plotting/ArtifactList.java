package us.ihmc.graphics3DDescription.yoGraphics.plotting;

import java.util.ArrayList;

import us.ihmc.graphics3DDescription.plotting.artifact.Artifact;

public class ArtifactList
{
   private String label;
   private ArrayList<Artifact> artifacts;

   public ArtifactList(String label, ArrayList<Artifact> artifacts)
   {
      // TODO: should a defensive copy be made of artifacts?
      this.label = label;
      updateLabels(artifacts);
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
      updateLabels(artifacts);
   }

   public ArtifactList(String label, Artifact artifact)
   {
      this.label = label;
      updateLabel(artifact);

      ArrayList<Artifact> artifacts = new ArrayList<Artifact>(1);

      artifacts.add(artifact);

      this.artifacts = artifacts;
   }

   public ArtifactList(String label)
   {
      this.label = label;
      this.artifacts = new ArrayList<Artifact>();
   }

   private void updateLabels(ArrayList<Artifact> artifacts)
   {
      for (Artifact artifact : artifacts)
      {
         updateLabel(artifact);
      }
   }

   private void updateLabel(Artifact artifact)
   {
      artifact.setLabel(label);
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
      updateLabel(artifact);
      this.artifacts.add(artifact);
   }

   public void addAll(ArrayList<Artifact> artifacts)
   {
      updateLabels(artifacts);
      this.artifacts.addAll(artifacts);
   }

   public void addArtifactsToPlotter(PlotterInterface plotter)
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
