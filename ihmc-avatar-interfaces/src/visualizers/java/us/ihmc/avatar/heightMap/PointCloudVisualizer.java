package us.ihmc.avatar.heightMap;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class PointCloudVisualizer extends AnimationTimer
{
   private static final double POINT_SIZE = 0.015;
   private static final double MINMAX_XY = 5.0;
   private final Group rootNode = new Group();

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private final AtomicReference<Pair<Mesh, Material>> pointCloudToRender = new AtomicReference<>();
   private final MeshView meshView = new MeshView();

   private final ExecutorService pointCloudUpdater = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final RigidBodyTransform transform = new RigidBodyTransform();

   public PointCloudVisualizer()
   {
      rootNode.getChildren().add(meshView);
      transform.getRotation().setToPitchOrientation(Math.toRadians(45.0));
   }

   @Override
   public void handle(long now)
   {
      Pair<Mesh, Material> pointCloudMesh = pointCloudToRender.getAndSet(null);
      if (pointCloudMesh != null)
      {
         meshView.setMesh(pointCloudMesh.getKey());
         meshView.setMaterial(pointCloudMesh.getValue());
      }
   }

   public void processPointCloud(PointCloudData pointCloudData)
   {
      pointCloudUpdater.execute(() -> processPointCloudInternal(pointCloudData));
   }

   private final AtomicBoolean isProcessing = new AtomicBoolean(false);

   private void processPointCloudInternal(PointCloudData pointCloudData)
   {
      if (isProcessing.getAndSet(true))
      {
         return;
      }

      /* Compute mesh */
      meshBuilder.clear();
      Point3D[] pointCloud = pointCloudData.getPointCloud();
      for (int i = 0; i < pointCloud.length; i++)
      {
         if (pointCloud[i] != null && Math.abs(pointCloud[i].getX()) < MINMAX_XY && Math.abs(pointCloud[i].getY()) < MINMAX_XY)
         {
            Point3D point = new Point3D(pointCloud[i]);
            point.applyTransform(transform);

            meshBuilder.addCube(POINT_SIZE, point, Color.RED);
         }
      }

      pointCloudToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
      isProcessing.set(false);
   }

   @Override
   public void stop()
   {
      super.stop();
      pointCloudUpdater.shutdownNow();
   }

   public Group getRoot()
   {
      return rootNode;
   }
}
