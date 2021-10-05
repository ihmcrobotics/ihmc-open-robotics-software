package us.ihmc.avatar.heightMap;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import org.apache.commons.lang3.tuple.Pair;
import org.opencv.core.Mat;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class PointCloudVisualizer extends AnimationTimer
{
   private static final double POINT_SIZE = 0.015;
   private static final double MIN_X = -0.5;
   private static final double MAX_X = 3.25;
   private static final double MIN_Y = -1.25;
   private static final double MAX_Y = 1.25;
   private static final double MIN_Z = -10.0;
   private static final double MAX_Z = 0.0;
   private final Group rootNode = new Group();

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private final AtomicReference<Pair<Mesh, Material>> pointCloudToRender = new AtomicReference<>();
   private final MeshView meshView = new MeshView();
   private final PoseReferenceFrame ousterFrame = new PoseReferenceFrame("ousterFrame", ReferenceFrame.getWorldFrame());

   private final ExecutorService pointCloudUpdater = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));

   public PointCloudVisualizer(Messager messager)
   {
      rootNode.getChildren().add(meshView);
//      transform.getRotation().setToPitchOrientation(Math.toRadians(30.0));
//      transform.getTranslation().set(-0.2, -0.15, 1.0);

      messager.registerTopicListener(HeightMapMessagerAPI.PointCloudData, this::processPointCloud);
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

   public void processPointCloud(Pair<PointCloud2, FramePose3D> pointCloudData)
   {
      pointCloudUpdater.execute(() -> processPointCloudInternal(pointCloudData));
   }

   private final AtomicBoolean isProcessing = new AtomicBoolean(false);

   private void processPointCloudInternal(Pair<PointCloud2, FramePose3D> pointCloudData)
   {
      if (isProcessing.getAndSet(true))
      {
         return;
      }

      /* Compute mesh */
      meshBuilder.clear();

      PointCloudData pointCloud = new PointCloudData(pointCloudData.getKey(), 1000000, false);
      ousterFrame.setPoseAndUpdate(pointCloudData.getRight());

      // Transform ouster data
      for (int i = 0; i < pointCloud.getPointCloud().length; i++)
      {
         FramePoint3D point = new FramePoint3D(ousterFrame, pointCloud.getPointCloud()[i]);
         point.changeFrame(ReferenceFrame.getWorldFrame());
         pointCloud.getPointCloud()[i].set(point);

         if (MathTools.intervalContains(point.getX(), MIN_X, MAX_X) &&
             MathTools.intervalContains(point.getY(), MIN_Y, MAX_Y) &&
             MathTools.intervalContains(point.getZ(), MIN_Z, MAX_Z))
            meshBuilder.addCube(POINT_SIZE, point, Color.RED);
      }

      pointCloudToRender.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
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
