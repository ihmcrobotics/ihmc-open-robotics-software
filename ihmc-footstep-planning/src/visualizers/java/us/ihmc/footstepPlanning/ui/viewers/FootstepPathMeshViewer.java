package us.ihmc.footstepPlanning.ui.viewers;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import jdk.nashorn.internal.ir.annotations.Ignore;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.IDLSequence.Double;
import us.ihmc.jMonkeyEngineToolkit.tralala.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class FootstepPathMeshViewer extends AnimationTimer
{
   private final Group root = new Group();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();

   private final double[] defaultWaypointProportions = new double[]{0.15, 0.85};
   private final AtomicReference<Boolean> showSolution;
   private final AtomicReference<Boolean> showIntermediatePlan;
   private final AtomicReference<FootstepDataListMessage> footstepDataListMessage;
   private final AtomicReference<Boolean> ignorePartialFootholds;
   private final AtomicBoolean solutionWasReceived = new AtomicBoolean(false);
   private final AtomicBoolean reset = new AtomicBoolean(false);
   private final AtomicBoolean renderShiftedFootsteps = new AtomicBoolean(false);

   private final MeshView footstepPathMeshView = new MeshView();
   private final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   public FootstepPathMeshViewer(Messager messager)
   {
      ignorePartialFootholds = messager.createInput(IgnorePartialFootholdsTopic, false);
      footstepDataListMessage = messager.createInput(FootstepPlanResponseTopic, null);

      messager.registerTopicListener(FootstepPlanResponseTopic, footstepPlan -> executorService.submit(() -> {
         solutionWasReceived.set(true);
         processFootstepPath(footstepPlan);
      }));

      messager.registerTopicListener(NodeDataTopic, nodeData -> executorService.submit(() -> {
         solutionWasReceived.set(false);
         processLowestCostNodeList(nodeData);
      }));

      messager.registerTopicListener(RenderShiftedWaypointsTopic, value ->
      {
         renderShiftedFootsteps.set(value);
         processFootstepPath(footstepDataListMessage.get());
      });

      messager.registerTopicListener(IgnorePartialFootholdsTopic, b -> processFootstepPath(footstepDataListMessage.get()));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePathTopic, data -> reset.set(true));

      showSolution = messager.createInput(ShowFootstepPlanTopic, true);
      showIntermediatePlan = messager.createInput(ShowNodeDataTopic, true);
   }

   private void processLowestCostNodeList(FootstepNodeDataListMessage message)
   {
      if (message.getIsFootstepGraph())
         return;

      IDLSequence.Object<FootstepNodeDataMessage> nodeDataList = message.getNodeData();
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      for (int i = 0; i < nodeDataList.size(); i++)
      {
         addNodeDataToFootstepPlan(footstepDataListMessage, nodeDataList.get(i));
      }

      processFootstepPath(footstepDataListMessage);
   }

   private static void addNodeDataToFootstepPlan(FootstepDataListMessage footstepDataListMessage, FootstepNodeDataMessage nodeData)
   {
      RigidBodyTransform footstepPose = new RigidBodyTransform();
      footstepPose.setRotationYawAndZeroTranslation(nodeData.getYawIndex() * LatticeNode.gridSizeYaw);
      footstepPose.setTranslationX(nodeData.getXIndex() * LatticeNode.gridSizeXY);
      footstepPose.setTranslationY(nodeData.getYIndex() * LatticeNode.gridSizeXY);

      FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().add();
      RigidBodyTransform snapTransform = new RigidBodyTransform();
      snapTransform.set(nodeData.getSnapRotation(), nodeData.getSnapTranslation());
      snapTransform.transform(footstepPose);
      footstepDataMessage.getLocation().set(footstepPose.getTranslationVector());
      footstepDataMessage.getOrientation().set(footstepPose.getRotationMatrix());
      footstepDataMessage.setRobotSide(nodeData.getRobotSide());
   }

   private synchronized void processFootstepPath(FootstepDataListMessage footstepDataListMessage)
   {
      meshBuilder.clear();

      FramePose3D footPose = new FramePose3D();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = new ConvexPolygon2D();
      FootstepPlan plan = FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage);

      if(ignorePartialFootholds.get())
      {
         for (int i = 0; i < plan.getNumberOfSteps(); i++)
         {
            plan.getFootstep(i).setFoothold(defaultContactPoints.get(plan.getFootstep(i).getRobotSide()));
         }
      }

     for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         Color regionColor = getFootstepColor(footstepDataListMessage.getFootstepDataList().get(i));

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
      }

      meshReference.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   private Color getFootstepColor(FootstepDataMessage footstepDataMessage)
   {
      if(renderShiftedFootsteps.get())
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
      else if (solutionWasReceived.get())
      {
         return footstepDataMessage.getRobotSide() == 0 ? Color.RED : Color.GREEN;
      }
      else
      {
         return Color.GRAY;
      }
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
      boolean addIntermediatePlan = showIntermediatePlan.get() && !solutionWasReceived.get() && root.getChildren().isEmpty();
      boolean addFinalPlan = showSolution.get() && solutionWasReceived.get() && root.getChildren().isEmpty();
      if (addIntermediatePlan || addFinalPlan)
         root.getChildren().add(footstepPathMeshView);

      boolean removeIntermediatePlan = !showIntermediatePlan.get() && !solutionWasReceived.get() && !root.getChildren().isEmpty();
      boolean removeFinalPlan = !showSolution.get() && solutionWasReceived.get() && !root.getChildren().isEmpty();
      if (removeIntermediatePlan || removeFinalPlan)
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

   public void setDefaultContactPoints(RobotContactPointParameters<RobotSide> defaultContactPointParameters)
   {
      SegmentDependentList<RobotSide, ArrayList<Point2D>> controllerFootGroundContactPoints = defaultContactPointParameters.getControllerFootGroundContactPoints();
      for(RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         ArrayList<Point2D> defaultContactPoints = controllerFootGroundContactPoints.get(robotSide);
         for (int i = 0; i < defaultContactPoints.size(); i++)
         {
            defaultFoothold.addVertex(defaultContactPoints.get(i));
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
}
