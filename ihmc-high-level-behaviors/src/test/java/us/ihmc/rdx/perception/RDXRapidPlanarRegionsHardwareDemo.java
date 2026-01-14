package us.ihmc.rdx.perception;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.robotDataLogger.ZEDSDKAnnounce;
import us.ihmc.robotDataLogger.logger.ZEDSVOLoggerManager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

import java.util.LinkedList;
import java.util.Queue;

public class RDXRapidPlanarRegionsHardwareDemo
{
   private static final int ZED_FPS = 30;

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private Mat depthU16C1Image;
   private RDXPose3DGizmo zedPoseGizmo = new RDXPose3DGizmo();
   private RDXRawImagePointCloudVisualizer pointCloudVisualizer;
   private RDXRapidRegionsUI rapidRegionsUI = new RDXRapidRegionsUI();
   private RapidPlanarRegionsExtractor rapidRegionsExtractor;
   private final ZEDImageSensor zedImageSensor;
   private final RepeatingTaskThread zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);
   private final Queue<RawImage> depthImageQueue = new LinkedList<>();
   private final Queue<RawImage> colorImageQueue = new LinkedList<>();
   private final ROS2Node announceNode = new ROS2NodeBuilder().build("zed_announce_node");
   private final RepeatingTaskThread zedSDKAnnounceThread;

   public RDXRapidPlanarRegionsHardwareDemo()
   {
      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_2, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_NEURAL);
      zedImageSensor.run(true);

      ROS2Publisher<ZEDSDKAnnounce> publisher = announceNode.createPublisher(ZEDSVOLoggerManager.ZED_SDK_ANNOUNCE_TOPIC);
      zedSDKAnnounceThread = new RepeatingTaskThread("ZEDSDKAnnounceThread", () ->
      {
         // Pack controller timestamp only if we recieved robot configuation data in the past 1/50th of a second
         // We want to align the ZED clock with the controller clock
         // Doing this association on board the robot is the most accurate
         // This avoids delays and significant time stretching issues we experienced
         if (zedImageSensor.isSensorRunning())
         {
            ZEDSDKAnnounce message = new ZEDSDKAnnounce();
            message.setSensorName("AlexExperimentalZEDXMini");
            message.setAddress("10.42.0.1");
            message.setPort((short) zedImageSensor.getStreamingPort());
            message.setFps(zedImageSensor.getFps());
            message.setBitrate(zedImageSensor.getStreamingBitrate());
            message.setSensorTimestamp(zedImageSensor.getLastGrabTimestamp());
//            if (syncedRobot.getDataReceptionTimerSnapshot().isRunning(0.02))
//               message.setControllerTimestamp(syncedRobot.getLatestRobotConfigurationData().getMonotonicTime());
//            else
//               message.setControllerTimestamp(0);
            publisher.publish(message);
         }

      });
      zedSDKAnnounceThread.setFrequencyLimit(5.0);
      zedSDKAnnounceThread.startRepeating();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);
            baseUI.getImGuiPanelManager().addPanel("Point Cloud", pointCloudVisualizer::renderImGuiWidgets);
            zedGrabThread.startRepeating();

            robotInteractableReferenceFrame = new RDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(2.2, 0.0, 1.0);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            zedPoseGizmo = new RDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            zedPoseGizmo.create(baseUI.getPrimary3DPanel());
            zedPoseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(zedPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(zedPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(zedPoseGizmo, RDXSceneLevel.VIRTUAL);
            zedPoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(10.0));
         }

         @Override
         public void render()
         {
            pointCloudVisualizer.update();

            // TODO update rapid regions and render

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            zedGrabThread.blockingKill();
            zedImageSensor.close();
            pointCloudVisualizer.destroy();
            colorImageQueue.forEach(RawImage::release);
            depthImageQueue.forEach(RawImage::release);

            baseUI.dispose();
         }
      });
   }

   private void zedGrabThread() throws InterruptedException
   {
      zedImageSensor.waitForGrab();

      RawImage colorImage = zedImageSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      RawImage depthImage = zedImageSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

      colorImageQueue.add(colorImage);
      depthImageQueue.add(depthImage);

      RawImage oldestColorImage = colorImageQueue.peek();
      while (oldestColorImage != null)
      {
         pointCloudVisualizer.setColorImage(oldestColorImage);
         colorImageQueue.remove().release();
         oldestColorImage = colorImageQueue.peek();
      }

      RawImage oldestDepthImage = depthImageQueue.peek();
      while (oldestDepthImage != null)
      {
         pointCloudVisualizer.setDepthImage(oldestDepthImage);
         depthImageQueue.remove().release();
         oldestDepthImage = depthImageQueue.peek();
      }
   }

   public static void main(String[] args)
   {
      new RDXRapidPlanarRegionsHardwareDemo();
   }
}
