package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.jMonkeyEngineToolkit.tralala.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlan;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;


public class FootstepPathMeshViewer extends AnimationTimer
{
   private final Group root = new Group();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicReference<Boolean> showSolution;
   private final AtomicBoolean solutionWasReceived = new AtomicBoolean(false);
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final MeshView footstepPathMeshView = new MeshView();
   private final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   public static final QuadrantDependentList<Color> solutionFootstepColors = new QuadrantDependentList<>(Color.GREEN, Color.RED, Color.DARKGREEN, Color.DARKRED);
   private static final QuadrantDependentList<Color> intermediateFootstepColors = new QuadrantDependentList<>(Color.rgb(160, 160, 160), Color.rgb(160, 160, 160), Color.rgb(160, 160, 160), Color.rgb(160, 160, 160));

   public FootstepPathMeshViewer(Messager messager)
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepPlanTopic, footstepPlan -> executorService.submit(() -> {
         solutionWasReceived.set(true);
         processFootstepPath(footstepPlan);
      }));



      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePathTopic, data -> reset.set(true));

      showSolution = messager.createInput(FootstepPlannerMessagerAPI.ShowFootstepPlanTopic, true);
   }


   private synchronized void processFootstepPath(FootstepPlan plan)
   {
      meshBuilder.clear();
      QuadrantDependentList<Color> colors = solutionWasReceived.get() ? solutionFootstepColors : intermediateFootstepColors;

      FramePoint3D footPosition = new FramePoint3D();

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         QuadrupedTimedStep footstep = plan.getFootstep(i);
         Color regionColor = colors.get(footstep.getRobotQuadrant());

         footstep.getGoalPosition(footPosition);
         footPosition.addZ(0.01);

         meshBuilder.addSphere(0.05, footPosition, regionColor);
      }

      meshReference.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   @Override
   public void handle(long now)
   {
      boolean addFinalPlan = showSolution.get() && solutionWasReceived.get() && root.getChildren().isEmpty();
      if (addFinalPlan)
         root.getChildren().add(footstepPathMeshView);

      boolean removeFinalPlan = !showSolution.get() && solutionWasReceived.get() && !root.getChildren().isEmpty();
      if (removeFinalPlan)
         root.getChildren().clear();

      if (reset.getAndSet(false))
      {
         footstepPathMeshView.setMesh(null);
         footstepPathMeshView.setMaterial(null);
         meshReference.set(null);
         return;
      }

      Pair<Mesh, Material> newMeshAndMaterial = meshReference.getAndSet(null);
      if (newMeshAndMaterial != null)
      {
         footstepPathMeshView.setMesh(newMeshAndMaterial.getKey());
         footstepPathMeshView.setMaterial(newMeshAndMaterial.getValue());
      }
   }

   @Override
   public void stop()
   {
      super.stop();
      executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }
}
