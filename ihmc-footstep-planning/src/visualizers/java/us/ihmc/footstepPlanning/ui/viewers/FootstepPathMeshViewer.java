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
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.idl.IDLSequence.Double;
import us.ihmc.jMonkeyEngineToolkit.tralala.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class FootstepPathMeshViewer extends AnimationTimer
{
   private static Color footWaypointColor = Color.WHITE;
   private static Color midpointColor = Color.GREEN;
   private static double footWaypointRadius = 0.02;
   private static double midpointRadius = 0.02;

   private final Group root = new Group();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();

   private final double[] defaultWaypointProportions = new double[]{0.15, 0.85};
   private final AtomicReference<Boolean> showSolution;
   private final AtomicReference<Boolean> showPostProcessingInfo;
   private final AtomicReference<FootstepDataListMessage> footstepDataListMessage;
   private final AtomicReference<Boolean> ignorePartialFootholds;
   private final AtomicReference<Pose3DReadOnly> leftFootPose;
   private final AtomicReference<Pose3DReadOnly> rightFootPose;
   private final AtomicBoolean solutionWasReceived = new AtomicBoolean(false);
   private final AtomicBoolean reset = new AtomicBoolean(false);
   private final AtomicBoolean renderShiftedFootsteps = new AtomicBoolean(false);

   private final MeshView footstepPathMeshView = new MeshView();
   private final MeshView footstepPostProcessedMeshView = new MeshView();
   private final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
   private final AtomicReference<Pair<Mesh, Material>> postProcessedMeshReference = new AtomicReference<>(null);
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private final JavaFXMultiColorMeshBuilder postProcessedMeshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   public FootstepPathMeshViewer(Messager messager)
   {
      ignorePartialFootholds = messager.createInput(IgnorePartialFootholds, false);
      footstepDataListMessage = messager.createInput(FootstepPlanResponse, null);
      leftFootPose = messager.createInput(LeftFootPose, null);
      rightFootPose = messager.createInput(RightFootPose, null);

      messager.registerTopicListener(FootstepPlanResponse, footstepPlan -> executorService.submit(() -> {
         solutionWasReceived.set(true);
         processFootstepPath(footstepPlan);
      }));

      messager.registerTopicListener(RenderShiftedWaypoints, value ->
      {
         renderShiftedFootsteps.set(value);
         processFootstepPath(footstepDataListMessage.get());
      });

      messager.registerTopicListener(IgnorePartialFootholds, b -> processFootstepPath(footstepDataListMessage.get()));
      messager.registerTopicListener(ComputePath, data -> reset.set(true));
      messager.registerTopicListener(GlobalReset, data -> reset.set(true));

      showSolution = messager.createInput(ShowFootstepPlan, true);
      showPostProcessingInfo = messager.createInput(ShowPostProcessingInfo, true);
   }

   private synchronized void processFootstepPath(FootstepDataListMessage footstepDataListMessage)
   {
      if (footstepDataListMessage == null)
      {
         return;
      }

      meshBuilder.clear();

      FramePose3D footPose = new FramePose3D();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = new ConvexPolygon2D();
      FootstepPlan plan = FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage);

      if (ignorePartialFootholds.get())
      {
         for (int i = 0; i < plan.getNumberOfSteps(); i++)
         {
            plan.getFootstep(i).setFoothold(defaultContactPoints.get(plan.getFootstep(i).getRobotSide()));
         }
      }

      boolean hasInfoToRenderFootsteps = leftFootPose.get() != null && rightFootPose.get() != null;

      FramePoint3D stanceFootPosition = new FramePoint3D();
      FramePoint3D previousStanceFootPosition = new FramePoint3D();
      RobotSide firstStepSide = plan.getFootstep(0).getRobotSide();
      if (hasInfoToRenderFootsteps && firstStepSide == RobotSide.LEFT)
      {
         previousStanceFootPosition.set(leftFootPose.get().getPosition());
         stanceFootPosition.set(rightFootPose.get().getPosition());
      }
      else if (hasInfoToRenderFootsteps)
      {
         previousStanceFootPosition.set(rightFootPose.get().getPosition());
         stanceFootPosition.set(leftFootPose.get().getPosition());
      }

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().get(i);
         Color regionColor = getFootstepColor(footstepDataMessage);

         footstep.getSoleFramePose(footPose);
         footPose.get(transformToWorld);
         transformToWorld.appendTranslation(0.0, 0.0, 0.01);

         if (footstep.hasFoothold())
            footstep.getFoothold(foothold);
         else
            foothold.set(defaultContactPoints.get(plan.getFootstep(i).getRobotSide()));

         Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
         {
            vertices[j] = new Point2D(foothold.getVertex(j));
         }

         meshBuilder.addMultiLine(transformToWorld, vertices, 0.01, regionColor, true);
         meshBuilder.addPolygon(transformToWorld, foothold, regionColor);

         for (Point3D waypoint : footstepDataMessage.getCustomPositionWaypoints())
         {
            postProcessedMeshBuilder.addSphere(footWaypointRadius, waypoint, footWaypointColor);
         }

         if (hasInfoToRenderFootsteps && footstepDataMessage.getTransferWeightDistribution() != -1.0)
         {
            FramePoint3D copMidpoint = new FramePoint3D();
            copMidpoint.interpolate(previousStanceFootPosition, stanceFootPosition, footstepDataMessage.getTransferWeightDistribution());
            postProcessedMeshBuilder.addSphere(midpointRadius, copMidpoint, getWaypointColor(footstepDataMessage));
         }

         previousStanceFootPosition.set(stanceFootPosition);
         stanceFootPosition.set(footPose.getPosition());
      }

      meshReference.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
      postProcessedMeshReference.set(new Pair<>(postProcessedMeshBuilder.generateMesh(), postProcessedMeshBuilder.generateMaterial()));
   }

   private Color getFootstepColor(FootstepDataMessage footstepDataMessage)
   {
      if (renderShiftedFootsteps.get())
      {
         if(hasDefaultWaypointProportions(footstepDataMessage))
         {
            // default waypoints
            return Color.GRAY;
         }
         else
         {
            double epsilon = 1e-5;
            if(EuclidCoreTools.epsilonEquals(defaultWaypointProportions[0], footstepDataMessage.getCustomWaypointProportions().get(0), epsilon))
            {
               // second waypoint shifted forward
               return Color.GREEN;
            }
            else
            {
               // first waypoint shifted back
               return Color.RED;
            }
         }
      }

      return footstepDataMessage.getRobotSide() == 0 ? Color.RED : Color.GREEN;
   }

   private Color getWaypointColor(FootstepDataMessage footstepDataMessage)
   {
      return toTransparentColor(footstepDataMessage.getRobotSide() == 0 ? Color.RED : Color.GREEN, 0.5);
   }

   private boolean hasDefaultWaypointProportions(FootstepDataMessage footstepDataMessage)
   {
      double epsilon = 1e-5;
      Double customWaypointProportions = footstepDataMessage.getCustomWaypointProportions();
      if(customWaypointProportions.isEmpty())
         return true;
      else
         return EuclidCoreTools.epsilonEquals(customWaypointProportions.get(0), defaultWaypointProportions[0], epsilon) && EuclidCoreTools
               .epsilonEquals(customWaypointProportions.get(1), defaultWaypointProportions[1], epsilon);
   }

   @Override
   public void handle(long now)
   {
      boolean addSolution = showSolution.get() && solutionWasReceived.get() && root.getChildren().isEmpty();
      if (addSolution)
         root.getChildren().add(footstepPathMeshView);
      boolean addPostProcessingInfo = showPostProcessingInfo.get() && addSolution;
      if (addPostProcessingInfo)
         root.getChildren().add(footstepPostProcessedMeshView);

      boolean removeSolution = !showSolution.get() && solutionWasReceived.get() && !root.getChildren().isEmpty();
      if (removeSolution)
         root.getChildren().clear();

      if (reset.getAndSet(false))
      {
         footstepPathMeshView.setMesh(null);
         footstepPathMeshView.setMaterial(null);
         footstepPostProcessedMeshView.setMesh(null);
         footstepPostProcessedMeshView.setMaterial(null);
         meshReference.set(null);
         postProcessedMeshReference.set(null);
         return;
      }

      Pair<Mesh, Material> newMeshAndMaterial = meshReference.getAndSet(null);
      if (newMeshAndMaterial != null)
      {
         footstepPathMeshView.setMesh(newMeshAndMaterial.getKey());
         footstepPathMeshView.setMaterial(newMeshAndMaterial.getValue());
      }

      Pair<Mesh, Material> newPostProcessedMeshAndMaterial = postProcessedMeshReference.getAndSet(null);
      if (newPostProcessedMeshAndMaterial != null)
      {
         footstepPostProcessedMeshView.setMesh(newPostProcessedMeshAndMaterial.getKey());
         footstepPostProcessedMeshView.setMaterial(newPostProcessedMeshAndMaterial.getValue());
      }
   }

   public void setDefaultContactPoints(SideDependentList<List<Point2D>> defaultContactPoints)
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         for (int i = 0; i < defaultContactPoints.get(robotSide).size(); i++)
         {
            defaultFoothold.addVertex(defaultContactPoints.get(robotSide).get(i));
         }

         defaultFoothold.update();
         this.defaultContactPoints.put(robotSide, defaultFoothold);
      }
   }

   public void setDefaultWaypointProportions(double[] defaultWaypointProportions)
   {
      this.defaultWaypointProportions[0] = defaultWaypointProportions[0];
      this.defaultWaypointProportions[1] = defaultWaypointProportions[1];
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

   private static Color toTransparentColor(Color opaqueColor, double opacity)
   {
      double red = opaqueColor.getRed();
      double green = opaqueColor.getGreen();
      double blue = opaqueColor.getBlue();
      return new Color(red, green, blue, opacity);
   }
}
