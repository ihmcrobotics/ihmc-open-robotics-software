package us.ihmc.footstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXVisualizers.MeshHolder;
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
   private static final Color idealStepColor = Color.color(0.0, 0.0, 1.0, 0.4);
   private static final Color leftFootColor = Color.color(0.9, 0.05, 0.05, 0.9);
   private static final Color rightFootColor = Color.color(0.0, 0.52, 0.0, 0.9);

   private final List<Point2D> defaultFootPoints = new ArrayList<>();
   private final ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D();

   private final Group root = new Group();
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette());

   private final MeshHolder startOfSwingStepGraphic = new MeshHolder(root);
   private final MeshHolder stanceStepGraphic = new MeshHolder(root);
   private final MeshHolder loggedUnsnappedCandidateStepGraphic = new MeshHolder(root);
   private final MeshHolder loggedSnappedCandidateStepGraphic = new MeshHolder(root);
   private final MeshHolder loggedWiggledCandidateStepGraphic = new MeshHolder(root);
   private final MeshHolder loggedIealStepGraphic = new MeshHolder(root);

   private final AtomicReference<Pair<DiscreteFootstep, FootstepSnapData>> startOfSwingStepToVisualize;
   private final AtomicReference<Pair<DiscreteFootstep, FootstepSnapData>> stanceStepToVisualize;
   private final AtomicReference<Pair<DiscreteFootstep, FootstepSnapData>> touchdownStepToVisualize;
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

      startOfSwingStepToVisualize = messager.createInput(FootstepPlannerMessagerAPI.StartOfSwingStepToVisualize);
      stanceStepToVisualize = messager.createInput(FootstepPlannerMessagerAPI.StanceStepToVisualize);
      touchdownStepToVisualize = messager.createInput(FootstepPlannerMessagerAPI.TouchdownStepToVisualize);
      idealStep = messager.createInput(FootstepPlannerMessagerAPI.LoggedIdealStep);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLogGraphics, root::setVisible);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLoggedStartOfSwingStep, startOfSwingStepGraphic.getMeshView()::setVisible);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLoggedStanceStep, stanceStepGraphic.getMeshView()::setVisible);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLoggedUnsnappedCandidateStep, loggedUnsnappedCandidateStepGraphic.getMeshView()::setVisible);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLoggedSnappedCandidateStep, loggedSnappedCandidateStepGraphic.getMeshView()::setVisible);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLoggedWiggledCandidateStep, loggedWiggledCandidateStepGraphic.getMeshView()::setVisible);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLoggedIdealStep, loggedIealStepGraphic.getMeshView()::setVisible);

      root.setVisible(false);
   }

   @Override
   public void handle(long now)
   {
      try
      {
         handleInteral();
      }
      catch (Exception e)
      {
         // don't hard crash if RuntimeException is thrown
         e.printStackTrace();
      }
   }

   private void handleInteral()
   {
      Pair<DiscreteFootstep, FootstepSnapData> startOfSwingStepData = this.startOfSwingStepToVisualize.getAndSet(null);
      if (startOfSwingStepData != null)
      {
         generateFootstepMesh(startOfSwingStepData, startOfSwingStepGraphic);
      }

      Pair<DiscreteFootstep, FootstepSnapData> stanceStepData = this.stanceStepToVisualize.getAndSet(null);
      if (stanceStepData != null)
      {
         generateFootstepMesh(stanceStepData, stanceStepGraphic);
      }

      Pair<DiscreteFootstep, FootstepSnapData> touchdownStepData = this.touchdownStepToVisualize.getAndSet(null);
      if (touchdownStepData != null)
      {
         DiscreteFootstep touchdownStep = touchdownStepData.getLeft();
         FootstepSnapData snapData = touchdownStepData.getRight();

         RigidBodyTransform stepPose = new RigidBodyTransform();
         RigidBodyTransform snapTransform = snapData.getSnapTransform();
         RigidBodyTransform wiggleTransformInWorld = snapData.getWiggleTransformInWorld();

         if (snapTransform.containsNaN())
         {
            stepPose.setTranslationAndIdentityRotation(touchdownStep.getX(), touchdownStep.getY(), 0.1);
            stepPose.getRotation().setYawPitchRoll(touchdownStep.getYaw(), 0.0, 0.0);
         }
         else
         {
            DiscreteFootstepTools.getSnappedStepTransform(touchdownStep, snapTransform, stepPose);
            double yaw = stepPose.getRotation().getYaw();
            stepPose.getRotation().setYawPitchRoll(yaw, 0.0, 0.0);
            stepPose.appendTranslation(0.0, 0.0, 0.1);
         }

         // Render unsnapped footstep
         meshBuilder.clear();
         addFootstep(stepPose.getTranslation(), stepPose.getRotation(), defaultFootPoints, defaultFootPolygon, unsnappedFootholdColor);
         loggedUnsnappedCandidateStepGraphic.setMeshReference(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         loggedUnsnappedCandidateStepGraphic.update();

         // Render snapped footstep if wiggle transform is present
         if (!wiggleTransformInWorld.containsNaN() && !wiggleTransformInWorld.epsilonEquals(new RigidBodyTransform(), 1e-6))
         {
            meshBuilder.clear();
            DiscreteFootstepTools.getSnappedStepTransform(touchdownStep, snapTransform, stepPose);
            addFootstep(stepPose.getTranslation(), stepPose.getRotation(), defaultFootPoints, defaultFootPolygon, snappedFootholdColor);
            loggedSnappedCandidateStepGraphic.setMeshReference(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
            loggedSnappedCandidateStepGraphic.update();
         }

         // Render snap and wiggled footstep
         meshBuilder.clear();
         RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform();

         if (!snapData.getSnapTransform().containsNaN())
         {
            snapData.packSnapAndWiggleTransform(snapAndWiggleTransform);
         }

         DiscreteFootstepTools.getSnappedStepTransform(touchdownStep, snapAndWiggleTransform, stepPose);

         ConvexPolygon2D croppedFoothold = snapData.getCroppedFoothold();
         List<Point2D> footPolygon = croppedFoothold.getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());
         Color candidateStepColor = touchdownStep.getRobotSide() == RobotSide.LEFT ? leftFootColor : rightFootColor;

         if (footPolygon.isEmpty() || croppedFoothold.containsNaN())
         {
            addFootstep(stepPose.getTranslation(), stepPose.getRotation(), defaultFootPoints, defaultFootPolygon, candidateStepColor);
         }
         else
         {
            addFootstep(stepPose.getTranslation(), stepPose.getRotation(), footPolygon, croppedFoothold, candidateStepColor);
         }

         this.loggedWiggledCandidateStepGraphic.setMeshReference(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         loggedWiggledCandidateStepGraphic.update();
      }

      // Render ideal step
      RigidBodyTransform idealStep = this.idealStep.getAndSet(null);
      if (idealStep != null)
      {
         meshBuilder.clear();
         addFootstep(idealStep.getTranslation(), idealStep.getRotation(), defaultFootPoints, defaultFootPolygon, idealStepColor);
         this.loggedIealStepGraphic.setMeshReference(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
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

   private void generateFootstepMesh(Pair<DiscreteFootstep, FootstepSnapData> footstepData, MeshHolder meshHolder)
   {
      meshBuilder.clear();
      DiscreteFootstep footstep = footstepData.getLeft();
      FootstepSnapData footstepSnapData = footstepData.getRight();

      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform();
      RigidBodyTransform snappedNodeTransform = new RigidBodyTransform();
      footstepSnapData.packSnapAndWiggleTransform(snapAndWiggleTransform);
      DiscreteFootstepTools.getSnappedStepTransform(footstep, snapAndWiggleTransform, snappedNodeTransform);

      List<Point2D> footPoints = footstepSnapData.getCroppedFoothold().getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());
      Color stanceStepColor = footstep.getRobotSide() == RobotSide.LEFT ? leftFootColor : rightFootColor;

      if (footPoints.isEmpty())
      {
         addFootstep(snappedNodeTransform.getTranslation(), snappedNodeTransform.getRotation(), defaultFootPoints, defaultFootPolygon, stanceStepColor);
      }
      else
      {
         addFootstep(snappedNodeTransform.getTranslation(),
                     snappedNodeTransform.getRotation(),
                     footPoints,
                     footstepSnapData.getCroppedFoothold(),
                     stanceStepColor);
      }

      meshHolder.setMeshReference(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
      meshHolder.update();
   }

   public Group getRoot()
   {
      return root;
   }
}
