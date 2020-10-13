package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrameState;

import java.util.List;
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

   private final REAUIMessager uiMessager;

   public SLAMFrameStateViewer(REAUIMessager uiMessager, Topic<Boolean> enableTopic, Topic<Boolean> clearTopic, Topic<Integer> sizeTopic)
   {
      this.uiMessager = uiMessager;

      newMessageToRender = uiMessager.createInput(SLAMModuleAPI.IhmcSLAMFrameState);

      enable = uiMessager.createInput(enableTopic, false);

      uncorrectedPointCloudViewer = new StereoVisionPointCloudViewer(SLAMModuleAPI.UIStereoSLAMPointCloud, uiMessager, enableTopic, clearTopic, sizeTopic);

      root.getChildren().addAll(uncorrectedPointCloudViewer.getRoot());
   }

   public void run()
   {
      if (!enable.get())
         return;

      if (newMessageToRender.get() == null)
         return;

      unpackPointCloud(newMessageToRender.getAndSet(null));
      uncorrectedPointCloudViewer.run();
   }

   public void render()
   {
      uncorrectedPointCloudViewer.render();
   }

   public void clear()
   {
      uncorrectedPointCloudViewer.clear();
   }

   public Node getRoot()
   {
      return root;
   }

   private StereoVisionPointCloudMessage createLatestFrameStereoVisionPointCloudMessage(Point3DReadOnly[] uncorrectedPointCloudBuffer,
                                                                                        Point3DReadOnly[] correspondingPointCloudBuffer,
                                                                                        List<? extends Point3DReadOnly> correctedPointCloudBuffer)
   {
      int numberOfPointsToPack = uncorrectedPointCloudBuffer.length + correspondingPointCloudBuffer.length + correctedPointCloudBuffer.size();

      Point3D[] pointCloudBuffer = new Point3D[numberOfPointsToPack];
      int[] colorBuffer = new int[numberOfPointsToPack];
      for (int i = 0; i < uncorrectedPointCloudBuffer.length; i++)
      {
         pointCloudBuffer[i] = new Point3D(uncorrectedPointCloudBuffer[i]);
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(LATEST_ORIGINAL_POINT_CLOUD_COLOR);
      }
      for (int i = uncorrectedPointCloudBuffer.length; i < uncorrectedPointCloudBuffer.length + correspondingPointCloudBuffer.length; i++)
      {
         pointCloudBuffer[i] = new Point3D(correspondingPointCloudBuffer[i - uncorrectedPointCloudBuffer.length]);
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(SOURCE_POINT_CLOUD_COLOR);
      }
      for (int i = uncorrectedPointCloudBuffer.length + correspondingPointCloudBuffer.length; i < numberOfPointsToPack; i++)
      {
         pointCloudBuffer[i] = new Point3D(correctedPointCloudBuffer.get(i - uncorrectedPointCloudBuffer.length - correspondingPointCloudBuffer.length));
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(LATEST_POINT_CLOUD_COLOR);
      }
      return PointCloudCompression.compressPointCloud(19870612L, pointCloudBuffer, colorBuffer, numberOfPointsToPack, 0.001, null);
   }

   private void unpackPointCloud(SLAMFrameState message)
   {
      Point3DReadOnly[] uncorrectedPointCloudBuffer = message.getUncorrectedPointCloudInWorld();
      List<? extends Point3DReadOnly> correctedPointCloudBuffer = message.getCorrectedPointCloudInWorld();
      Point3DReadOnly[] correspondingPointCloudBuffer = message.setCorrespondingPointsInWorld();

      StereoVisionPointCloudMessage convertedMessage = createLatestFrameStereoVisionPointCloudMessage(uncorrectedPointCloudBuffer, correspondingPointCloudBuffer, correctedPointCloudBuffer);

      uiMessager.broadcastMessage(SLAMModuleAPI.UIStereoSLAMPointCloud, convertedMessage);
   }
}
