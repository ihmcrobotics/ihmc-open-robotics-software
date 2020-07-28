package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrameState;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PointCloudAnchorPaneController;

import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

public class SLAMFrameStateViewer implements Runnable
{
   private static final Color LATEST_ORIGINAL_POINT_CLOUD_COLOR = Color.BEIGE;
   private static final Color SOURCE_POINT_CLOUD_COLOR = Color.RED;
   private static final Color LATEST_POINT_CLOUD_COLOR = Color.LIME;

   private final Group root = new Group();

   private final AtomicReference<SLAMFrameState> newMessageToRender;

   private final AtomicReference<Boolean> enable;

   private final StereoVisionPointCloudViewer uncorrectedPointCloudViewer;
   private final StereoVisionPointCloudViewer correctedPointCloudViewer;
   private final StereoVisionPointCloudViewer correspondingPointCloudViewer;

   private final REAUIMessager uiMessager;

   public SLAMFrameStateViewer(REAUIMessager uiMessager, Topic<Boolean> enableTopic, Topic<Boolean> clearTopic, Topic<Integer> sizeTopic)
   {
      this.uiMessager = uiMessager;

      newMessageToRender = uiMessager.createInput(SLAMModuleAPI.IhmcSLAMFrameState);

      enable = uiMessager.createInput(enableTopic, false);

      uncorrectedPointCloudViewer = new StereoVisionPointCloudViewer(SLAMModuleAPI.UIUncorrectedPointCloud, uiMessager, enableTopic, clearTopic, sizeTopic);
      correctedPointCloudViewer = new StereoVisionPointCloudViewer(SLAMModuleAPI.UICorrectedPointCloud, uiMessager, enableTopic, clearTopic, sizeTopic);
      correspondingPointCloudViewer = new StereoVisionPointCloudViewer(SLAMModuleAPI.UICorrespondingPointsInWorld,
                                                                       uiMessager,
                                                                       enableTopic,
                                                                       clearTopic,
                                                                       sizeTopic);

      root.getChildren().addAll(uncorrectedPointCloudViewer.getRoot(), correctedPointCloudViewer.getRoot(), correspondingPointCloudViewer.getRoot());
   }

   public void run()
   {
      if (!enable.get())
         return;

      if (newMessageToRender.get() == null)
         return;

      unpackPointCloud(newMessageToRender.getAndSet(null));
      uncorrectedPointCloudViewer.run();
      correctedPointCloudViewer.run();
      correspondingPointCloudViewer.run();
   }

   public void render()
   {
      uncorrectedPointCloudViewer.render();
      correctedPointCloudViewer.render();
      correspondingPointCloudViewer.render();
   }

   public void clear()
   {
      uncorrectedPointCloudViewer.clear();
      correctedPointCloudViewer.clear();
      correspondingPointCloudViewer.clear();
   }

   public Node getRoot()
   {
      return root;
   }

   private void unpackPointCloud(SLAMFrameState message)
   {
      Point3DReadOnly[] uncorrectedPointCloudBuffer = message.getUncorrectedPointCloudInWorld();
      Point3DReadOnly[] correctedPointCloudBuffer = message.getCorrectedPointCloudInWorld();
      Point3DReadOnly[] correspondingPointCloudBuffer = message.setCorrespondingPointsInWorld();

      int uncorrectedSize = uncorrectedPointCloudBuffer.length;
      int correctedSize = correctedPointCloudBuffer.length;
      int correspondingSize = correspondingPointCloudBuffer.length;

      int[] uncorrectedColorBuffer = new int[uncorrectedSize];
      int[] correctedColorBuffer = new int[correctedSize];
      int[] correspondingColorBuffer = new int[correspondingSize];

      for (int i = 0; i < uncorrectedSize; i++)
         uncorrectedColorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(LATEST_ORIGINAL_POINT_CLOUD_COLOR);
      for (int i = 0; i < correctedSize; i++)
         correctedColorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(LATEST_POINT_CLOUD_COLOR);
      for (int i = 0; i < correspondingSize; i++)
         correspondingColorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(SOURCE_POINT_CLOUD_COLOR);

      StereoVisionPointCloudMessage uncorrectedMessage = PointCloudCompression.compressPointCloud(19870612L,
                                                                                                  uncorrectedPointCloudBuffer,
                                                                                                  uncorrectedColorBuffer,
                                                                                                  uncorrectedSize,
                                                                                                  0.001,
                                                                                                  null);
      StereoVisionPointCloudMessage correctedMessage = PointCloudCompression.compressPointCloud(19870612L,
                                                                                                correctedPointCloudBuffer,
                                                                                                correctedColorBuffer,
                                                                                                correctedSize,
                                                                                                0.001,
                                                                                                null);
      StereoVisionPointCloudMessage correspondingMessage = PointCloudCompression.compressPointCloud(19870612L,
                                                                                                    correspondingPointCloudBuffer,
                                                                                                    correspondingColorBuffer,
                                                                                                    correspondingSize,
                                                                                                    0.001,
                                                                                                    null);

      uiMessager.broadcastMessage(SLAMModuleAPI.UIUncorrectedPointCloud, uncorrectedMessage);
      uiMessager.broadcastMessage(SLAMModuleAPI.UICorrectedPointCloud, correctedMessage);
      uiMessager.broadcastMessage(SLAMModuleAPI.UICorrespondingPointsInWorld, correspondingMessage);
   }
}
