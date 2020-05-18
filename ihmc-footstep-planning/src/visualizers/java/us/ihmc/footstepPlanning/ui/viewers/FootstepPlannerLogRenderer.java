package us.ihmc.footstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class FootstepPlannerLogRenderer extends AnimationTimer
{
   private static final Color unsnappedFootholdColor = Color.color(0.5, 0.5, 0.5, 0.4);
   private static final Color snappedFootholdColor = Color.color(0.82, 0.41, 0.12, 0.9);
   private static final Color snapWiggleCroppedFootholdColor = Color.color(0.9, 0.05, 0.05, 0.9);
   private static final Color idealStepColor = Color.color(0.0, 0.0, 1.0, 0.4);
   private static final Color stanceStepColor = Color.color(0.0, 0.52, 0.0, 0.9);

   private final List<Point2D> defaultFootPoints = new ArrayList<>();
   private final ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D();

   private final Group root = new Group();
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette());

   private final MeshHolder loggedStanceStepGraphic = new MeshHolder(root);
   private final MeshHolder loggedCandidateStepGraphic = new MeshHolder(root);
   private final MeshHolder loggedIealStepGraphic = new MeshHolder(root);

   private final AtomicReference<Pair<RigidBodyTransform, ConvexPolygon2D>> stanceStep;
   private final AtomicReference<FootstepPlannerEdgeData> candidateStep;
   private final AtomicReference<RigidBodyTransform> idealStep;

   public FootstepPlannerLogRenderer(SideDependentList<List<Point2D>> defaultContactPoints, Messager messager)
   {
      if (defaultContactPoints == null)
      {
         defaultFootPolygon.set(PlannerTools.createDefaultFootPolygon());
      }
      else
      {
         List<Point2D> contactPoints = defaultContactPoints.get(RobotSide.LEFT);
         contactPoints.forEach(defaultFootPolygon::addVertex);
         defaultFootPolygon.update();
      }

      for (int i = 0; i < defaultFootPolygon.getNumberOfVertices(); i++)
      {
         defaultFootPoints.add(new Point2D(defaultFootPolygon.getVertex(i)));
      }

      stanceStep = messager.createInput(FootstepPlannerMessagerAPI.LoggedStanceStepToVisualize);
      candidateStep = messager.createInput(FootstepPlannerMessagerAPI.LoggedCandidateStepToVisualize);
      idealStep = messager.createInput(FootstepPlannerMessagerAPI.LoggedIdealStep);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLogGraphics, root::setVisible);

      root.setVisible(false);
   }

   @Override
   public void handle(long now)
   {
      Pair<RigidBodyTransform, ConvexPolygon2D> stanceStepData = this.stanceStep.getAndSet(null);
      if (stanceStepData != null)
      {
         meshBuilder.clear();
         List<Point2D> footPoints = stanceStepData.getRight().getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());
         if (footPoints.isEmpty())
         {
            addFootstep(stanceStepData.getLeft().getTranslation(), stanceStepData.getLeft().getRotation(), defaultFootPoints, defaultFootPolygon, stanceStepColor);
         }
         else
         {
            addFootstep(stanceStepData.getLeft().getTranslation(), stanceStepData.getLeft().getRotation(), footPoints, stanceStepData.getRight(), stanceStepColor);
         }

         this.loggedStanceStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         loggedStanceStepGraphic.update();

         meshBuilder.clear();
         this.loggedCandidateStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         loggedCandidateStepGraphic.update();
      }

      FootstepPlannerEdgeData candidateStepData = this.candidateStep.getAndSet(null);
      if (candidateStepData != null)
      {
         meshBuilder.clear();
         FootstepNode candidateNode = candidateStepData.getCandidateNode();

         RigidBodyTransform projectedFootPose = new RigidBodyTransform();
         RigidBodyTransform snapTransform = candidateStepData.getCandidateNodeSnapData().getSnapTransform();
         RigidBodyTransform wiggleTransformInWorld = candidateStepData.getCandidateNodeSnapData().getWiggleTransformInWorld();

         if (snapTransform.containsNaN())
         {
            projectedFootPose.setTranslationAndIdentityRotation(candidateNode.getX(), candidateNode.getY(), 0.1);
            projectedFootPose.getRotation().setYawPitchRoll(candidateNode.getYaw(), 0.0, 0.0);
         }
         else
         {
            FootstepNodeTools.getSnappedNodeTransform(candidateNode, snapTransform, projectedFootPose);
            double yaw = projectedFootPose.getRotation().getYaw();
            projectedFootPose.getRotation().setYawPitchRoll(yaw, 0.0, 0.0);
            projectedFootPose.appendTranslation(0.0, 0.0, 0.1);
         }

         addFootstep(projectedFootPose.getTranslation(), projectedFootPose.getRotation(), defaultFootPoints, defaultFootPolygon, unsnappedFootholdColor);

         if (!wiggleTransformInWorld.containsNaN() && !wiggleTransformInWorld.epsilonEquals(new RigidBodyTransform(), 1e-6))
         {
            FootstepNodeTools.getSnappedNodeTransform(candidateNode, snapTransform, projectedFootPose);
            addFootstep(projectedFootPose.getTranslation(), projectedFootPose.getRotation(), defaultFootPoints, defaultFootPolygon, snappedFootholdColor);
         }

         RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform();
         candidateStepData.getCandidateNodeSnapData().packSnapAndWiggleTransform(snapAndWiggleTransform);
         FootstepNodeTools.getSnappedNodeTransform(candidateNode, snapAndWiggleTransform, projectedFootPose);

         ConvexPolygon2D croppedFoothold = candidateStepData.getCandidateNodeSnapData().getCroppedFoothold();
         List<Point2D> footPolygon = croppedFoothold.getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());

         if (footPolygon.isEmpty() || croppedFoothold.containsNaN())
         {
            addFootstep(projectedFootPose.getTranslation(), projectedFootPose.getRotation(), defaultFootPoints, defaultFootPolygon, snapWiggleCroppedFootholdColor);
         }
         else
         {
            addFootstep(projectedFootPose.getTranslation(), projectedFootPose.getRotation(), footPolygon, croppedFoothold, snapWiggleCroppedFootholdColor);
         }

         this.loggedCandidateStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         loggedCandidateStepGraphic.update();
      }

      RigidBodyTransform idealStep = this.idealStep.getAndSet(null);
      if (idealStep != null)
      {
         meshBuilder.clear();
         addFootstep(idealStep.getTranslation(), idealStep.getRotation(), defaultFootPoints, defaultFootPolygon, idealStepColor);
         this.loggedIealStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         loggedIealStepGraphic.update();
      }
   }

   private void addFootstep(Tuple3DReadOnly translation, Orientation3DReadOnly orientation, List<Point2D> footPoints, ConvexPolygon2D footPolygon, Color color)
   {
      RigidBodyTransform transform = new RigidBodyTransform(orientation, translation);
      transform.appendTranslation(0.0, 0.0, 0.0025);
      meshBuilder.addMultiLine(transform, footPoints, 0.01, color, true);
      meshBuilder.addPolygon(transform, footPolygon, color);
   }

   public Group getRoot()
   {
      return root;
   }
}
