package us.ihmc.footstepPlanning.ui.viewers;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.jMonkeyEngineToolkit.tralala.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class FootstepPostProcessingMeshViewer extends AnimationTimer
{
   private static Color footWaypointColor = Color.WHITE;
   private static Color midpointColor = Color.GREEN;
   private static double footWaypointRadius = 0.02;
   private static double midpointRadius = 0.02;

   private final Group root = new Group();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicReference<Boolean> showSolution;
   private final AtomicReference<Boolean> showPostProcessingInfo;
   private final AtomicReference<Point3D> leftFootPosition;
   private final AtomicReference<Point3D> rightFootPosition;
   private final AtomicBoolean solutionWasReceived = new AtomicBoolean(false);
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final MeshView footstepMeshView = new MeshView();
   private final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   public FootstepPostProcessingMeshViewer(Messager messager)
   {
      leftFootPosition = messager.createInput(LeftFootStartPosition, null);
      rightFootPosition = messager.createInput(RightFootStartPosition, null);

      messager.registerTopicListener(FootstepPlanResponse, footstepPlan -> executorService.submit(() -> {
         solutionWasReceived.set(true);
         processFootstepPath(footstepPlan);
      }));


      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePath, data -> reset.set(true));

      showSolution = messager.createInput(ShowFootstepPlan, true);
      showPostProcessingInfo = messager.createInput(ShowPostProcessingInfo, true);
   }

   private synchronized void processFootstepPath(FootstepDataListMessage footstepDataListMessage)
   {
      meshBuilder.clear();

      FramePose3D footPose = new FramePose3D();
      FootstepPlan plan = FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage);

      boolean hasInfoToRenderFootsteps = leftFootPosition.get() != null && rightFootPosition.get() != null;

      FramePoint3D stanceFootPosition = new FramePoint3D();
      FramePoint3D previousStanceFootPosition = new FramePoint3D();
      RobotSide firstStepSide = plan.getFootstep(0).getRobotSide();
      if (hasInfoToRenderFootsteps && firstStepSide == RobotSide.LEFT)
      {
         previousStanceFootPosition.set(leftFootPosition.get());
         stanceFootPosition.set(rightFootPosition.get());
      }
      else if (hasInfoToRenderFootsteps)
      {
         previousStanceFootPosition.set(rightFootPosition.get());
         stanceFootPosition.set(leftFootPosition.get());
      }

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         plan.getFootstep(i).getSoleFramePose(footPose);
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().get(i);

         for (Point3D waypoint : footstepDataMessage.getCustomPositionWaypoints())
         {
            meshBuilder.addSphere(footWaypointRadius, waypoint, footWaypointColor);
         }

         if (hasInfoToRenderFootsteps && footstepDataMessage.getTransferWeightDistribution() != -1.0)
         {
            FramePoint3D copMidpoint = new FramePoint3D();
            copMidpoint.interpolate(previousStanceFootPosition, stanceFootPosition, footstepDataMessage.getTransferWeightDistribution());
            meshBuilder.addSphere(midpointRadius, copMidpoint, midpointColor);
         }

         previousStanceFootPosition.set(stanceFootPosition);
         stanceFootPosition.set(footPose.getPosition());
      }

      meshReference.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   @Override
   public void handle(long now)
   {
      boolean addFinalPlan = showSolution.get() && solutionWasReceived.get() && showPostProcessingInfo.get() && root.getChildren().isEmpty();
      if (addFinalPlan)
         root.getChildren().add(footstepMeshView);

      boolean removeFinalPlan = (!showSolution.get() || !showPostProcessingInfo.get()) && solutionWasReceived.get() && !root.getChildren().isEmpty();
      if (removeFinalPlan)
         root.getChildren().clear();

      if (reset.getAndSet(false))
      {
         footstepMeshView.setMesh(null);
         footstepMeshView.setMaterial(null);
         meshReference.set(null);
         return;
      }

      Pair<Mesh, Material> newMeshAndMaterial = meshReference.getAndSet(null);
      if (newMeshAndMaterial != null)
      {
         footstepMeshView.setMesh(newMeshAndMaterial.getKey());
         footstepMeshView.setMaterial(newMeshAndMaterial.getValue());
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
