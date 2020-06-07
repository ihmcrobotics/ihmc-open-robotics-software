package us.ihmc.robotEnvironmentAwareness.slam.viewer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.javaFXVisualizers.FootstepMeshManager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepMeshViewer implements Runnable
{
   private final static int MAXIMUM_NUMBER_OF_FOOTSTEPS_TO_VIZ = 200;

   private final Group root = new Group();

   private final JavaFXMultiColorMeshBuilder meshBuilder;
   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();

   private final AtomicInteger numberOfFootstepsToRender = new AtomicInteger();
   private final AtomicReference<FootstepDataMessage> newFootstepDataMessage;
   private final List<FootstepMeshManager> footstepMesheManagers = new ArrayList<>();

   private final AtomicReference<Boolean> enable;

   public FootstepMeshViewer(REAUIMessager uiMessager)
   {
      newFootstepDataMessage = uiMessager.createInput(SLAMModuleAPI.FootstepDataState);

      Topic<Boolean> showviz = SLAMModuleAPI.ShowFootstepDataViz;
      enable = uiMessager.createInput(showviz, true);

      uiMessager.registerTopicListener(SLAMModuleAPI.ClearFootstepDataViz, (c) -> clear());

      uiMessager.registerTopicListener(SLAMModuleAPI.SLAMClear, (c) -> clear());

      meshBuilder = new JavaFXMultiColorMeshBuilder(normalBasedColorPalette1D);

      for (int i = 0; i < MAXIMUM_NUMBER_OF_FOOTSTEPS_TO_VIZ; i++)
      {
         footstepMesheManagers.add(new FootstepMeshManager(root, meshBuilder, i));
      }

      uiMessager.registerTopicListener(showviz, show -> footstepMesheManagers.forEach(mesh -> mesh.getMeshHolder().getMeshView().setVisible(show)));
   }

   public void render()
   {
      int numberOfFootstepsToViz = numberOfFootstepsToRender.get();
      for (int i = 0; i < numberOfFootstepsToViz; i++)
      {
         footstepMesheManagers.get(i).computeMesh();
         footstepMesheManagers.get(i).updateMesh();
      }
   }

   public void setDefaultContactPoints(SideDependentList<List<Point2D>> defaultContactPoints)
   {
      for (int i = 0; i < footstepMesheManagers.size(); i++)
      {
         footstepMesheManagers.get(i).setFoothold(defaultContactPoints);
         footstepMesheManagers.get(i).setIgnorePartialFootHolds(true);
      }
   }

   public void addFootstepDataMessage(FootstepDataMessage footstepDataMessage)
   {
      int index = numberOfFootstepsToRender.getAndAdd(1);
      footstepMesheManagers.get(index).setFootstepDataMessage(footstepDataMessage);
   }

   @Override
   public void run()
   {
      if (!enable.get())
         return;

      if (newFootstepDataMessage.get() == null)
         return;

      addFootstepDataMessage(newFootstepDataMessage.getAndSet(null));
   }

   public void clear()
   {
      for (int i = 0; i < footstepMesheManagers.size(); i++)
      {
         footstepMesheManagers.get(i).clear();
      }
      numberOfFootstepsToRender.set(0);
   }

   public Node getRoot()
   {
      return root;
   }
}