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
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.avatar.heightMap.HeightMapUpdater.APPROX_OUSTER_TRANSFORM;
import static us.ihmc.avatar.heightMap.HeightMapUpdater.USE_OUSTER_FRAME;

public class PointCloudVisualizer extends AnimationTimer
{
   private static final double POINT_SIZE = 0.015;
   private final Group rootNode = new Group();

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private final AtomicReference<Pair<Mesh, Material>> pointCloudToRender = new AtomicReference<>();
   private final MeshView meshView = new MeshView();
   private final PoseReferenceFrame ousterFrame = new PoseReferenceFrame("ousterFrame", ReferenceFrame.getWorldFrame());

   private final AtomicReference<Point2D> gridCenter = new AtomicReference<>(new Point2D());
   private final double gridSizeXY;

   private final ExecutorService pointCloudUpdater = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));

   public PointCloudVisualizer(Messager messager, HeightMapParameters parameters)
   {
      rootNode.getChildren().add(meshView);
      messager.registerTopicListener(HeightMapMessagerAPI.PointCloudData, this::processPointCloud);
      messager.registerTopicListener(HeightMapMessagerAPI.GridCenterX, x -> gridCenter.set(new Point2D(x, gridCenter.get().getY())));
      messager.registerTopicListener(HeightMapMessagerAPI.GridCenterY, y -> gridCenter.set(new Point2D(gridCenter.get().getX(), y)));
      gridSizeXY = parameters.getGridSizeXY();
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
      if (isProcessing.getAndSet(true))
      {
         return;
      }

      pointCloudUpdater.execute(() -> processPointCloudInternal(pointCloudData));
   }

   private final AtomicBoolean isProcessing = new AtomicBoolean(false);

   private void processPointCloudInternal(Pair<PointCloud2, FramePose3D> pointCloudData)
   {
      /* Compute mesh */
      meshBuilder.clear();

      PointCloudData pointCloud = new PointCloudData(pointCloudData.getKey(), 1000000, false);

      if (USE_OUSTER_FRAME)
      {
         ousterFrame.setPoseAndUpdate(pointCloudData.getRight());

         for (int i = 0; i < 64; i++)
         {
            int v = 10;
            for (int j = 0; j < 2048; j++)
            {
               FramePoint3D point = new FramePoint3D(ousterFrame, pointCloud.getPointCloud()[2048 * i + j]);
               point.changeFrame(ReferenceFrame.getWorldFrame());
               pointCloud.getPointCloud()[i].set(point);

               double alpha = i / 65.0;
               meshBuilder.addCube(POINT_SIZE, point, Color.RED.interpolate(Color.BLUE, alpha));
            }
         }

         //         for (int i = 0; i < pointCloud.getPointCloud().length; i++)
//         {
//            FramePoint3D point = new FramePoint3D(ousterFrame, pointCloud.getPointCloud()[i]);
//            point.changeFrame(ReferenceFrame.getWorldFrame());
//            pointCloud.getPointCloud()[i].set(point);
//
//            Point2D gridCenter = this.gridCenter.get();
//            double minX = gridCenter.getX() - 0.5 * gridSizeXY;
//            double maxX = gridCenter.getX() + 0.5 * gridSizeXY;
//            double minY = gridCenter.getY() - 0.5 * gridSizeXY;
//            double maxY = gridCenter.getY() + 0.5 * gridSizeXY;
//
//            if (MathTools.intervalContains(point.getX(), minX, maxX) && MathTools.intervalContains(point.getY(), minY, maxY))
//            {
//               meshBuilder.addCube(POINT_SIZE, point, Color.RED);
//            }
//         }
      }
      else
      {
         for (int i = 0; i < pointCloud.getPointCloud().length; i++)
         {
            pointCloud.getPointCloud()[i].applyTransform(APPROX_OUSTER_TRANSFORM);
         }
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
