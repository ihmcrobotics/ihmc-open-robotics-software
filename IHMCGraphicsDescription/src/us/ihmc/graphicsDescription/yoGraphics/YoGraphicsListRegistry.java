package us.ihmc.graphicsDescription.yoGraphics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.PlotterInterface;
import us.ihmc.tools.gui.GraphicsUpdatable;

public class YoGraphicsListRegistry
{
   private final ArrayList<YoGraphicsList> yoGraphicsLists = new ArrayList<YoGraphicsList>();
   private final ArrayList<ArtifactList> artifactLists = new ArrayList<ArtifactList>();
   
   private Object graphicsConch = null;
   private final ArrayList<GraphicsUpdatable> graphicsUpdatables = new ArrayList<GraphicsUpdatable>();
   private final ArrayList<GraphicsUpdatable> graphicsUpdatablesToUpdateInAPlaybackListener = new ArrayList<GraphicsUpdatable>();
   
   private boolean updateInSimulationThread = false;
   private boolean alreadyAddedToSimulationConstructionSet = false;
   private boolean alreadyAddedToPlotter = false;
   
   
   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final RigidBodyTransform simulatedRootToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform controllerWorldToRootTransform = new RigidBodyTransform();

   public YoGraphicsListRegistry()
   {
   }

   private void checkForRepeatNames(YoGraphicsList yoGraphicsList)
   {
      ArrayList<YoGraphic> yoGraphics = yoGraphicsList.getYoGraphics();

      for (YoGraphic yoGraphic : yoGraphics)
      {
         if (doesNameExistInYoGraphicsList(yoGraphic.getName()))
         {
            throw new RuntimeException("Repeat Name: " + yoGraphic.getName() + " in YoGraphic!");
         }
      }
   }

   private void checkForRepeatNames(ArtifactList artifactList)
   {
      ArrayList<Artifact> artifacts = artifactList.getArtifacts();

      for (Artifact artifact : artifacts)
      {
         if (doesNameExistInArtifactLists(artifact.getID()))
         {
            throw new RuntimeException("Repeat Name: " + artifact.getID() + " in ArtifactList!");
         }
      }
   }

   private boolean doesNameExistInYoGraphicsList(String nameToCheck)
   {
      for (YoGraphicsList yoGraphicsList : yoGraphicsLists)
      {
         ArrayList<YoGraphic> yoGraphics = yoGraphicsList.getYoGraphics();
         for (YoGraphic yoGraphic : yoGraphics)
         {
            String name = yoGraphic.getName();

            if (name.equals(nameToCheck))
            {
               return true;
            }
         }
      }

      return false;
   }

   private boolean doesNameExistInArtifactLists(String nameToCheck)
   {
      for (ArtifactList artifactList : artifactLists)
      {
         ArrayList<Artifact> artifacts = artifactList.getArtifacts();
         for (Artifact artifact : artifacts)
         {
            String name = artifact.getID();

            if (name.equals(nameToCheck))
            {
               return true;
            }
         }
      }

      return false;
   }

   public void registerYoGraphicsList(YoGraphicsList yoGraphicsList)
   {
      if (alreadyAddedToSimulationConstructionSet)
         throw new RuntimeException("Graphics have already been added to the SimulationConstructionSet. Cannot register more objects after this.");

      if (yoGraphicsLists.contains(yoGraphicsList))
      {
         throw new RuntimeException("Already registered YoGraphicsList " + yoGraphicsList);
      }

      checkForRepeatNames(yoGraphicsList);

      for (YoGraphicsList list : yoGraphicsLists)
      {
         if (list.getLabel().equals(yoGraphicsList.getLabel()))
         {
            // Combine them:
            ArrayList<YoGraphic> yoGraphics = yoGraphicsList.getYoGraphics();
            list.addAll(yoGraphics);
            return;
         }
      }

      yoGraphicsLists.add(yoGraphicsList);
   }

   public void registerArtifactList(ArtifactList artifactList)
   {
      if (alreadyAddedToSimulationConstructionSet)
         throw new RuntimeException("Graphics have already been added to the SimulationConstructionSet. Cannot register more objects after this.");

      if (artifactLists.contains(artifactList))
      {
         throw new RuntimeException("Already registered artifactList " + artifactList);
      }

      checkForRepeatNames(artifactList);

      for (ArtifactList list : artifactLists)
      {
         if (list.getLabel().equals(artifactList.getLabel()))
         {
            // Combine them:
            list.addAll(artifactList.getArtifacts());

            return;
         }
      }

      artifactLists.add(artifactList);
   }

   public void getRegisteredDynamicGraphicObjectsLists(ArrayList<YoGraphicsList> yoGraphicsLists)
   {
      yoGraphicsLists.addAll(this.yoGraphicsLists);
   }

   public void getRegisteredArtifactLists(ArrayList<ArtifactList> artifactLists)
   {
      artifactLists.addAll(this.artifactLists);
   }

   //TODO: Merge these graphics updatable things and get it to all work nicely.
   
   public void registerGraphicsUpdatableToUpdateInAPlaybackListener(GraphicsUpdatable graphicsUpdatable)
   {
      if (this.graphicsUpdatablesToUpdateInAPlaybackListener.contains(graphicsUpdatable))
         throw new RuntimeException("Already registered graphics updateable!");
      
      this.graphicsUpdatablesToUpdateInAPlaybackListener.add(graphicsUpdatable);
   }
   
   public ArrayList<GraphicsUpdatable> getGraphicsUpdatablesToUpdateInAPlaybackListener()
   {
      return graphicsUpdatablesToUpdateInAPlaybackListener;
   }
   
   public void addGraphicsUpdatable(GraphicsUpdatable graphicsUpdatable)
   {
      if (graphicsUpdatables.contains(graphicsUpdatable))
         return;

      graphicsUpdatables.add(graphicsUpdatable);
   }

   public void addGraphicsUpdatables(List<? extends GraphicsUpdatable> graphicsUpdatables)
   {
      for (int i = 0; i < graphicsUpdatables.size(); i++)
      {
         addGraphicsUpdatable(graphicsUpdatables.get(i));
      }
   }

   public void update()
   {
      if (updateInSimulationThread)
      {
         throw new RuntimeException("YoGraphics are already updated in the simulation thread.");
      }
      if (graphicsConch != null)
      {
         synchronized (graphicsConch)
         {
            updateRootTransform();
            for (int i = 0; i < graphicsUpdatables.size(); i++)
            {
               graphicsUpdatables.get(i).update();
            }
         }
      }
   }

   public void addArtifactListsToPlotter(PlotterInterface plotter)
   {
      if (alreadyAddedToPlotter)
      {
         throw new RuntimeException("Already added this ArtifactLists To Plotter: " + artifactLists);
      }

      if (artifactLists != null)
      {
         for (ArtifactList artifactList : artifactLists)
         {
            if (artifactList != null)
            {
               //               YoGraphicCheckBoxMenuItem checkBox = new YoGraphicCheckBoxMenuItem(graphicsList.getLabel(), graphicsList.getYoGraphics());
               //               yoGraphicMenu.add(checkBox);

               // add graphics to simulation individually
               artifactList.addArtifactsToPlotter(plotter);
            }
            else
               System.out.println("ArtifactList is null!");
         }
      }

      alreadyAddedToPlotter = true;

   }

   public void registerYoGraphicsLists(ArrayList<YoGraphicsList> yoGraphicsLists)
   {
      for (YoGraphicsList yoGraphicsList : yoGraphicsLists)
      {
         registerYoGraphicsList(yoGraphicsList);
      }
   }

   public void registerYoGraphic(String listName, YoGraphic yoGraphic)
   {
      YoGraphicsList list = new YoGraphicsList(listName, yoGraphic);
      registerYoGraphicsList(list);
   }

   public void registerYoGraphics(String listName, YoGraphic[] yoGraphics)
   {
      YoGraphicsList list = new YoGraphicsList(listName, yoGraphics);
      registerYoGraphicsList(list);
   }

   public void registerYoGraphics(String listName, ArrayList<? extends YoGraphic> yoGraphics)
   {
      YoGraphicsList list = new YoGraphicsList(listName, yoGraphics);
      registerYoGraphicsList(list);
   }

   public void registerArtifactLists(ArrayList<ArtifactList> artifactLists)
   {
      for (ArtifactList artifactList : artifactLists)
      {
         registerArtifactList(artifactList);
      }
   }

   public void registerArtifact(String listName, Artifact artifact)
   {
      ArtifactList list = new ArtifactList(listName, artifact);
      registerArtifactList(list);
   }

   public void registerArtifacts(String listName, Artifact[] artifacts)
   {
      ArtifactList list = new ArtifactList(listName, artifacts);
      registerArtifactList(list);
   }

   public void registerArtifacts(String listName, ArrayList<Artifact> artifacts)
   {
      ArtifactList list = new ArtifactList(listName, artifacts);
      registerArtifactList(list);
   }

   public void registerYoGraphicsAndArtifactsFromOtherRegistry(YoGraphicsListRegistry other)
   {
      registerYoGraphicsLists(other.yoGraphicsLists);
      registerArtifactLists(other.artifactLists);
   }

   public void hideYoGraphics()
   {
      int numberOfElements = yoGraphicsLists.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         YoGraphicsList yoGraphicsList = yoGraphicsLists.get(i);
         yoGraphicsList.hideYoGraphics();
      }
   }

   public void hideArtifacts()
   {
      int numberOfElements = artifactLists.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         ArtifactList artifactList = artifactLists.get(i);
         artifactList.hideArtifacts();
      }
   }

   public boolean checkAllYoGraphicsListAreShowing()
   {
      boolean ret = true;
      int numberOfElements = yoGraphicsLists.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         YoGraphicsList yoGraphicsList = yoGraphicsLists.get(i);
         ret = ret && yoGraphicsList.checkAllYoGraphicsAreShowing();
      }

      return ret;
   }

   public void setGraphicsConch(Object graphicsConch)
   {
      this.graphicsConch = graphicsConch;
   }

   public void setYoGraphicsUpdatedRemotely(boolean updatedRemotely)
   {
      this.updateInSimulationThread = updatedRemotely;
   }

   public List<YoGraphicsList> getYoGraphicsLists()
   {
      return yoGraphicsLists;
   }

   public void setYoGraphicsRegistered()
   {
      alreadyAddedToSimulationConstructionSet = true;
   }

   public boolean areYoGraphicsRegistered()
   {
      return alreadyAddedToSimulationConstructionSet;
   }
   
   
   private void updateRootTransform()
   {
           
      rootTransform.set(simulatedRootToWorldTransform);
      rootTransform.multiply(controllerWorldToRootTransform);
      
      for(int i = 0; i < yoGraphicsLists.size(); i++)
      {
         yoGraphicsLists.get(i).setRootTransform(rootTransform);
      }
      
   }
   
   /**
    * Set the transform from the root joint joint of the robot to the world, as known by the simulation. If both 
    * this and the controller transform are set, all elements are adjusted for the difference. 
    * 
    * @param transformToWorld
    */
   public void setSimulationTransformToWorld(RigidBodyTransform transformToWorld)
   {
      this.simulatedRootToWorldTransform.set(transformToWorld);
      if(updateInSimulationThread)
      {
         // This is cheap enough to do twice. This way, guarantee that data from the same tick is used.
         updateRootTransform();
      }
   }

   /**
    * Set the transform from the root joint joint of the robot to the world, as known by the controller. If both 
    * this and the simulation transform are set, all elements are adjusted for the difference.
    * 
    * @param transformToWorld
    */
   public void setControllerTransformToWorld(RigidBodyTransform transformToWorld)
   {
      this.controllerWorldToRootTransform.setAndInvert(transformToWorld);
      if(updateInSimulationThread)
      {
         updateRootTransform();
      }
   }
}
