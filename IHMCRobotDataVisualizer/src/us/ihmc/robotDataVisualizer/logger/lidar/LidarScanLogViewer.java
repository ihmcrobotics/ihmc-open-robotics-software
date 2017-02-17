package us.ihmc.robotDataVisualizer.logger.lidar;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.tools.thread.ThreadTools;

public class LidarScanLogViewer extends AnimationTimer
{
   private static final float SCAN_POINT_SIZE = 0.01f;
   private static final int NUMBER_OF_SCANS = 200;

   private static final Material defaultMaterial = new PhongMaterial(Color.DARKRED);

   private final Group root = new Group();
   private final Affine lidarPose = new Affine();
   private final Group scans = new Group();

   private final AtomicBoolean enabled = new AtomicBoolean(false);
   private final AtomicBoolean clearScan = new AtomicBoolean(false);
   private final AtomicInteger currentScanIndex = new AtomicInteger(0);

   private final AtomicReference<LidarScanMessage> newMessageToRender = new AtomicReference<>(null);
   private final AtomicReference<Affine> lastAffine = new AtomicReference<>();
   private final AtomicReference<MeshView> scanMeshToRender = new AtomicReference<>(null);
   private final JavaFXMultiColorMeshBuilder scanMeshBuilder;

   private final ExecutorService executor = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   public LidarScanLogViewer()
   {
      JavaFXCoordinateSystem lidarFrame = new JavaFXCoordinateSystem(0.1);
      lidarFrame.getTransforms().add(lidarPose);

      root.getChildren().addAll(lidarFrame, scans);

      TextureColorPalette1D scanColorPalette = new TextureColorPalette1D();
      scanColorPalette.setHueBased(1.0, 1.0);
      scanMeshBuilder = new JavaFXMultiColorMeshBuilder(scanColorPalette);
   }

   @Override
   public void handle(long now)
   {
      Affine affine = lastAffine.getAndSet(null);
      MeshView newScanMeshView = scanMeshToRender.getAndSet(null);

      if (clearScan.getAndSet(false))
      {
         scans.getChildren().clear();
         currentScanIndex.set(0);
      }

      if (!enabled.get())
         return;

      if (affine != null)
         lidarPose.setToTransform(affine);

      if (newScanMeshView != null)
      {
         ObservableList<Node> children = scans.getChildren();

         if (children.size() <= currentScanIndex.get())
            children.add(newScanMeshView);
         else
            children.set(currentScanIndex.get(), newScanMeshView);

         for (int i = currentScanIndex.get() + 1; i < currentScanIndex.get() + children.size(); i++)
            ((MeshView) children.get(i % children.size())).setMaterial(defaultMaterial);

         currentScanIndex.set((currentScanIndex.get() + 1) % NUMBER_OF_SCANS);
      }
   }

   private void computeScanMesh()
   {
      LidarScanMessage message = newMessageToRender.getAndSet(null);
      if (message == null)
         return;

      Point3D32 scanPoint = new Point3D32();
      scanMeshBuilder.clear();
      for (int i = 0; i < message.getNumberOfScanPoints(); i++)
      {
         double alpha = i / (double) message.getNumberOfScanPoints();
         Color color = Color.hsb(alpha * 240.0, 1.0, 1.0);

         message.getScanPoint(i, scanPoint);

         scanMeshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), scanPoint, color);
      }

      MeshView scanMeshView = new MeshView(scanMeshBuilder.generateMesh());
      scanMeshView.setMaterial(scanMeshBuilder.generateMaterial());
      scanMeshToRender.set(scanMeshView);
      scanMeshBuilder.clear();
   }

   public void renderLidarScanMessage(LidarScanMessage lidarScanMessage)
   {
      if (!enabled.get() || lidarScanMessage == null || newMessageToRender.get() != null)
         return;

      newMessageToRender.set(lidarScanMessage);

      Quaternion32 orientation = lidarScanMessage.getLidarOrientation();
      Point3D32 position = lidarScanMessage.getLidarPosition();
      lastAffine.set(JavaFXTools.createAffineFromQuaternionAndTuple(orientation, position));
      executor.execute(this::computeScanMesh);
   }

   @Override
   public void start()
   {
      enabled.set(true);
      super.start();
   }

   @Override
   public void stop()
   {
      enabled.set(false);
      executor.shutdownNow();
      super.stop();
   }

   public Group getRoot()
   {
      return root;
   }

   public void clearScans()
   {
      clearScan.set(true);
   }
}
