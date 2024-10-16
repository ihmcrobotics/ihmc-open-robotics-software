package us.ihmc.robotEnvironmentAwareness.slam.viewer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.javafx.FootstepMeshManager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.commons.robotics.robotSide.RobotSide;

public class FootstepMeshViewer extends AnimationTimer
{
   private final static int MAXIMUM_NUMBER_OF_FOOTSTEPS_TO_VIZ = 200;

   private final Group root = new Group();

   private final JavaFXMultiColorMeshBuilder meshBuilder;
   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();

   private final AtomicInteger numberOfFootstepsToRender = new AtomicInteger();
   private final AtomicReference<FootstepDataMessage> newFootstepDataMessage;
   private final List<FootstepMeshManager> footstepMesheManagers = new ArrayList<>();

   private final AtomicReference<Boolean> enable;
   private boolean isRunning = false;
   private final AtomicBoolean clearRequested = new AtomicBoolean(false);

   public FootstepMeshViewer(REAUIMessager uiMessager, Function<RobotSide, ConvexPolygon2D> contactPointsProvider)
   {
      newFootstepDataMessage = uiMessager.createInput(SLAMModuleAPI.FootstepDataState);

      Topic<Boolean> showviz = SLAMModuleAPI.ShowFootstepDataViz;
      enable = uiMessager.createInput(showviz, true);

      uiMessager.registerTopicListener(SLAMModuleAPI.ClearFootstepDataViz, (c) -> clear());

      uiMessager.registerTopicListener(SLAMModuleAPI.SLAMClear, (c) -> clear());

      meshBuilder = new JavaFXMultiColorMeshBuilder(normalBasedColorPalette1D);

      for (int i = 0; i < MAXIMUM_NUMBER_OF_FOOTSTEPS_TO_VIZ; i++)
      {
         int index = i;
         footstepMesheManagers.add(new FootstepMeshManager(root,
                                                           meshBuilder,
                                                           contactPointsProvider,
                                                           () -> numberOfFootstepsToRender.get() == (index + 1),
                                                           () -> false));
      }

      uiMessager.registerTopicListener(showviz, show -> Platform.runLater(() ->
      {
         footstepMesheManagers.forEach(mesh -> mesh.getMeshHolder().getMeshView().setVisible(show));
      }));
      uiMessager.registerModuleMessagerStateListener(isMessagerOpen ->
      {
         if (isMessagerOpen)
            start();
         else
            stop();
      });
   }

   private void render()
   {
      int numberOfFootstepsToViz = numberOfFootstepsToRender.get();
      for (int i = 0; i < numberOfFootstepsToViz; i++)
      {
         footstepMesheManagers.get(i).computeMesh();
         footstepMesheManagers.get(i).updateMesh();
      }
   }

   public void addFootstepDataMessage(FootstepDataMessage footstepDataMessage)
   {
      int index = numberOfFootstepsToRender.getAndAdd(1);
      footstepMesheManagers.get(index).setFootstepDataMessage(footstepDataMessage);
   }

   public void clear()
   {
      if (isRunning)
         clearRequested.set(true);
      else
         Platform.runLater(this::clearNow);
   }

   public Node getRoot()
   {
      return root;
   }

   @Override
   public void start()
   {
      super.start();
      isRunning = true;
   }

   @Override
   public void stop()
   {
      super.stop();
      isRunning = false;
   }

   @Override
   public void handle(long now)
   {
      if (clearRequested.getAndSet(false))
      {
         clearNow();
      }

      if (!enable.get())
         return;

      render();

      if (newFootstepDataMessage.get() == null)
         return;

      addFootstepDataMessage(newFootstepDataMessage.getAndSet(null));
   }

   private void clearNow()
   {
      for (int i = 0; i < footstepMesheManagers.size(); i++)
      {
         footstepMesheManagers.get(i).clear();
      }
      numberOfFootstepsToRender.set(0);
   }
}