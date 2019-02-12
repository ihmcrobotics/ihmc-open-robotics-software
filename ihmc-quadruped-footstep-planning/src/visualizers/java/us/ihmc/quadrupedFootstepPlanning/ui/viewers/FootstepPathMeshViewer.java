package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.idl.IDLSequence;
import us.ihmc.jMonkeyEngineToolkit.tralala.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlan;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;


public class FootstepPathMeshViewer extends AnimationTimer
{
   private static final ConvexPolygon2D defaultFootPolygon = PlannerTools.createDefaultFootPolygon();
   private final Group root = new Group();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicReference<Boolean> showSolution;
   private final AtomicReference<Boolean> showIntermediatePlan;
   private final AtomicBoolean solutionWasReceived = new AtomicBoolean(false);
   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final MeshView footstepPathMeshView = new MeshView();
   private final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   private static final SideDependentList<Color> solutionFootstepColors = new SideDependentList<>(Color.GREEN, Color.RED);
   private static final SideDependentList<Color> intermediateFootstepColors = new SideDependentList<>(Color.rgb(160, 160, 160), Color.rgb(160, 160, 160));

   public FootstepPathMeshViewer(Messager messager)
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepPlanTopic, footstepPlan -> executorService.submit(() -> {
         solutionWasReceived.set(true);
         processFootstepPath(footstepPlan);
      }));

      messager.registerTopicListener(FootstepPlannerMessagerAPI.NodeDataTopic, nodeData -> executorService.submit(() -> {
         solutionWasReceived.set(false);
         processLowestCostNodeList(nodeData);
      }));

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePathTopic, data -> reset.set(true));

      showSolution = messager.createInput(FootstepPlannerMessagerAPI.ShowFootstepPlanTopic, true);
      showIntermediatePlan = messager.createInput(FootstepPlannerMessagerAPI.ShowNodeDataTopic, true);
   }

   private void processLowestCostNodeList(FootstepNodeDataListMessage message)
   {
      if (message.getIsFootstepGraph())
         return;

      IDLSequence.Object<FootstepNodeDataMessage> nodeDataList = message.getNodeData();
      FootstepPlan footstepPlan = new FootstepPlan();
      for (int i = 0; i < nodeDataList.size(); i++)
      {
         addNodeDataToFootstepPlan(footstepPlan, nodeDataList.get(i));
      }

      processFootstepPath(footstepPlan);
   }

   private static void addNodeDataToFootstepPlan(FootstepPlan footstepPlan, FootstepNodeDataMessage nodeData)
   {
      RobotSide robotSide = RobotSide.fromByte(nodeData.getRobotSide());

      RigidBodyTransform footstepPose = new RigidBodyTransform();
      footstepPose.setRotationYawAndZeroTranslation(nodeData.getYawIndex() * FootstepNode.gridSizeYaw);
      footstepPose.setTranslationX(nodeData.getXIndex() * FootstepNode.gridSizeXY);
      footstepPose.setTranslationY(nodeData.getYIndex() * FootstepNode.gridSizeXY);

      RigidBodyTransform snapTransform = new RigidBodyTransform();
      snapTransform.set(nodeData.getSnapRotation(), nodeData.getSnapTranslation());
      snapTransform.transform(footstepPose);
      footstepPlan.addFootstep(robotSide, new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPose));
   }

   private synchronized void processFootstepPath(FootstepPlan plan)
   {
      meshBuilder.clear();
      SideDependentList<Color> colors = solutionWasReceived.get() ? solutionFootstepColors : intermediateFootstepColors;

      FramePose3D footPose = new FramePose3D();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = new ConvexPolygon2D();

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         Color regionColor = colors.get(footstep.getRobotSide());

         footstep.getSoleFramePose(footPose);
         footPose.get(transformToWorld);
         transformToWorld.appendTranslation(0.0, 0.0, 0.01);

         if (footstep.hasFoothold())
            footstep.getFoothold(foothold);
         else
            foothold.set(defaultFootPolygon);

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
