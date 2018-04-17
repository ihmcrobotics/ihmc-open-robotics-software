package us.ihmc.pathPlanning.visibilityGraphs.visualizer;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;

public class WalkerCollisionsViewer extends AnimationTimer
{
   private static final double COLLISION_SIZE = 0.1;
   private final Group root = new Group();
   private final AtomicReference<MeshView> collisionGraphics = new AtomicReference<>(null);
   private final AtomicReference<Boolean> reset;

   public WalkerCollisionsViewer(SimpleUIMessager messager)
   {
      messager.registerTopicListener(UIVisibilityGraphsTopics.WalkerCollisionLocations, this::processCollisions);
      reset = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
   }

   @Override
   public void handle(long now)
   {
      if (reset.getAndSet(false))
         root.getChildren().clear();

      MeshView meshView = collisionGraphics.getAndSet(null);

      if (meshView != null)
      {
         root.getChildren().add(meshView);
      }
   }

   private void processCollisions(List<Point3D> collisions)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Point3D collision : collisions)
         meshBuilder.addTetrahedron(COLLISION_SIZE, collision);

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(Color.RED));
      collisionGraphics.set(meshView);
   }

   public Node getRoot()
   {
      return root;
   }
}
