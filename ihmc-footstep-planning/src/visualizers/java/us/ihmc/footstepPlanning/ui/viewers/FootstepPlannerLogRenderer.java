package us.ihmc.footstepPlanning.ui.viewers;

import controller_msgs.msg.dds.FootstepPlanRequestPacket;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class FootstepPlannerLogRenderer extends AnimationTimer
{
   private final List<Point2D> defaultFootPoints = new ArrayList<>();
   private final ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D();

   private final Group root = new Group();
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette());
   private final MeshHolder debugParentStepGraphic = new MeshHolder();
   private final MeshHolder debugChildStepGraphic = new MeshHolder();
   private final MeshHolder debugIdealStepGraphic = new MeshHolder();

   private final AtomicReference<Pair<RigidBodyTransform, ConvexPolygon2D>> debugParentStep;
   private final AtomicReference<Pair<RigidBodyTransform, ConvexPolygon2D>> debugChildStep;
   private final AtomicReference<RigidBodyTransform> debugIdealStep;

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

      debugParentStep = messager.createInput(FootstepPlannerMessagerAPI.parentDebugStep);
      debugChildStep = messager.createInput(FootstepPlannerMessagerAPI.childDebugStep);
      debugIdealStep = messager.createInput(FootstepPlannerMessagerAPI.idealDebugStep);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLogGraphics, root::setVisible);

      root.setVisible(false);
   }

   @Override
   public void handle(long now)
   {
      Pair<RigidBodyTransform, ConvexPolygon2D> debugParentStep = this.debugParentStep.getAndSet(null);
      Pair<RigidBodyTransform, ConvexPolygon2D> debugChildStep = this.debugChildStep.getAndSet(null);
      RigidBodyTransform debugIdealStep = this.debugIdealStep.getAndSet(null);

      if (debugParentStep != null)
      {
         meshBuilder.clear();
         List<Point2D> footPoints = debugParentStep.getValue().getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());
         if (footPoints.isEmpty())
            addFootstep(debugParentStep.getKey().getTranslation(), debugParentStep.getKey().getRotation(), defaultFootPoints, defaultFootPolygon, Color.GREEN);
         else
            addFootstep(debugParentStep.getKey().getTranslation(), debugParentStep.getKey().getRotation(), footPoints, debugParentStep.getValue(), Color.GREEN);

         this.debugParentStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         debugParentStepGraphic.update();

         meshBuilder.clear();
         this.debugChildStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         debugChildStepGraphic.update();
      }

      if (debugChildStep != null)
      {
         meshBuilder.clear();
         List<Point2D> footPolygon = debugChildStep.getValue().getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());
         if (footPolygon.isEmpty() || debugChildStep.getValue().containsNaN())
            addFootstep(debugChildStep.getKey().getTranslation(), debugChildStep.getKey().getRotation(), defaultFootPoints, defaultFootPolygon, Color.RED);
         else
            addFootstep(debugChildStep.getKey().getTranslation(), debugChildStep.getKey().getRotation(), footPolygon, debugChildStep.getValue(), Color.RED);
         this.debugChildStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         debugChildStepGraphic.update();
      }

      if (debugIdealStep != null)
      {
         meshBuilder.clear();
         addFootstep(debugIdealStep.getTranslation(), debugIdealStep.getRotation(), defaultFootPoints, defaultFootPolygon, Color.BLUE);
         this.debugIdealStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         debugIdealStepGraphic.update();
      }
   }

   private void addFootstep(Tuple3DReadOnly translation, Orientation3DReadOnly orientation, List<Point2D> footPoints, ConvexPolygon2D footPolygon, Color color)
   {
      RigidBodyTransform transform = new RigidBodyTransform(orientation, translation);
      transform.appendTranslation(0.0, 0.0, 0.0025);
      meshBuilder.addMultiLine(transform, footPoints, 0.01, color, true);
      meshBuilder.addPolygon(transform, footPolygon, color);
   }

   class MeshHolder
   {
      final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
      final MeshView meshView = new MeshView();
      boolean addedFlag = false;

      void update()
      {
         Pair<Mesh, Material> mesh = meshReference.getAndSet(null);
         if (mesh != null)
         {
            if (!addedFlag)
            {
               root.getChildren().add(meshView);
               addedFlag = true;
            }

            meshView.setMesh(mesh.getKey());
            meshView.setMaterial(mesh.getValue());
         }
      }
   }

   public Group getRoot()
   {
      return root;
   }
}
