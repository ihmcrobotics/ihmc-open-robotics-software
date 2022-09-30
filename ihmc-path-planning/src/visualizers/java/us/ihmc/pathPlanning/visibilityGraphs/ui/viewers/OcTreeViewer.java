package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import perception_msgs.msg.dds.OcTreeKeyListMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class OcTreeViewer
{
   private static final boolean verbose = true;
   private static final Color ocTreeCellColor = Color.color(0.2, 0.4, 0.9, 0.6);

   private final Group root = new Group();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AnimationTimer animationTimer;

   private final AtomicBoolean enableRendering = new AtomicBoolean();
   private final AtomicReference<MeshView> meshToRender = new AtomicReference<>();
   private final AtomicBoolean meshBuildingLock = new AtomicBoolean();

   public OcTreeViewer()
   {
      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long l)
         {
            if (!enableRendering.get())
               return;

            MeshView mesh = meshToRender.getAndSet(null);
            if (mesh != null)
            {
               root.getChildren().clear();
               root.getChildren().add(mesh);
            }
         }
      };
   }

   public void start()
   {
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
      executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }

   public void setEnabled(boolean enabled)
   {
      enableRendering.set(enabled);
   }

   public void submitOcTreeData(OcTreeKeyListMessage ocTreeKeyListMessage)
   {
      if (verbose)
         LogTools.info("Received OcTree message. enabled = " + enableRendering.get() + ". resolution = " + ocTreeKeyListMessage.getTreeResolution());

      if (enableRendering.get() && !meshBuildingLock.get())
         executorService.submit(() -> buildMesh(ocTreeKeyListMessage));
   }

   private void buildMesh(OcTreeKeyListMessage ocTreeData)
   {
      meshBuildingLock.set(true);

      try
      {
         if (verbose)
            LogTools.info("Starting to build mesh...");

         JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

         List<OcTreeKey> ocTreeKeys = OcTreeMessageConverter.decompressMessage(ocTreeData.getKeys(), ocTreeData.getNumberOfKeys());
         double resolution = ocTreeData.getTreeResolution();
         int treeDepth = ocTreeData.getTreeDepth();

         for (int i = 0; i < ocTreeKeys.size(); i++)
         {
            OcTreeKey ocTreeKey = ocTreeKeys.get(i);
            Point3D nodePosition = OcTreeKeyConversionTools.keyToCoordinate(ocTreeKey, resolution, treeDepth);
            meshBuilder.addCube(resolution, nodePosition);
         }

         MeshView mesh = new MeshView(meshBuilder.generateMesh());
         mesh.setMaterial(new PhongMaterial(ocTreeCellColor));
         meshToRender.set(mesh);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      meshBuildingLock.set(false);
   }
}
