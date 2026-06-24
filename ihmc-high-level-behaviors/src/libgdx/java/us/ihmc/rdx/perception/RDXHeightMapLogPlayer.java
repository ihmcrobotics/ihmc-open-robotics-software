package us.ihmc.rdx.perception;

import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.HeightMapMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.gpuMapping.HeightMapExtractor;
import us.ihmc.perception.gpuMapping.HeightMapMessageTools;
import us.ihmc.perception.gpuMapping.HeightMapParameters;
import us.ihmc.perception.gpuMapping.worldModel.ChunkedMapManager;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2HeightMapVisualizer;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class RDXHeightMapLogPlayer
{
//   private static final String SVO_FILE = "/opt/ihmc/LogData/UserFolders/TomaszFolder/heightmap_test.svo2";
   private static final String SVO_FILE = "/opt/ihmc/LogData/UserFolders/DexFolder/FrameGrabber9000/01262026/ZED_Recording_58123737_20260126_152719.svo";

   private final RDXBaseUI baseUI;
   private final ROS2Node ros2Node;

   private final ROS2ZEDSVOPlaybackSensor zedPlaybackSensor;
   private final RDXZEDSVORecorderPanel zedSVOPanel;
   private RDXReferenceFrameGraphic zedFrameGraphic;

   private final RDXRawImagePointCloudVisualizer zedPointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud", true);
   private final RDXROS2HeightMapVisualizer heightMapVisualizer;

   private final ChunkedMapManager chunkedMapManager;
   private final HeightMapExtractor heightMapExtractor;
   private final HeightMapMessage heightMapMessage;
   private final ROS2Publisher<HeightMapMessage> heightMapMessagePublisher;
   private long heightMapSequenceId = 0;

   public RDXHeightMapLogPlayer()
   {
      ros2Node = new ROS2Node(getClass().getSimpleName());
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      baseUI = new RDXBaseUI();

      zedPlaybackSensor = new ROS2ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_X_MINI, zed.SL_DEPTH_MODE_NEURAL_LIGHT, SVO_FILE);
      zedPlaybackSensor.setTrackedPoseOffset(new Pose3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
      zedPlaybackSensor.useTrackedPose(true);
      BlockingQueue<RawImage> rawImageCollection = new LinkedBlockingQueue<>(ImageSensor.DEFAULT_IMAGE_QUEUE_CAPACITY);
      zedPlaybackSensor.registerImageQueue(rawImageCollection, ZEDImageSensor.DEPTH_IMAGE_KEY);
      zedSVOPanel = new RDXZEDSVORecorderPanel(ros2Helper);

      heightMapVisualizer = new RDXROS2HeightMapVisualizer("Height Map Visualizer");
      heightMapVisualizer.setupForImageMessage(ros2Helper);
      heightMapVisualizer.setupForChunkMessage(ros2Helper);

      HeightMapParameters heightMapParameters = new HeightMapParameters();
      chunkedMapManager = new ChunkedMapManager(ros2Node, heightMapParameters);

      heightMapExtractor = new HeightMapExtractor(heightMapParameters);
      heightMapMessage = new HeightMapMessage();
      heightMapMessagePublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_MESSAGE);

      RepeatingTaskThread imageThread = new RepeatingTaskThread("Image Thread", this::runMethod);
      imageThread.startRepeating();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            zedPointCloudVisualizer.create();
            zedPointCloudVisualizer.setActive(true);
            heightMapVisualizer.create();
            heightMapVisualizer.setActive(true);
            zedFrameGraphic = new RDXReferenceFrameGraphic(0.2);

            zedPlaybackSensor.run(true);

            baseUI.getImGuiPanelManager().addPanel("ZED SVO", zedSVOPanel::render);
            baseUI.getImGuiPanelManager().addPanel("ZED Point Cloud", zedPointCloudVisualizer::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel("Height Map Renderer", heightMapVisualizer::renderImGuiWidgets);

            baseUI.getPrimaryScene().addRenderableProvider(zedPointCloudVisualizer);
            baseUI.getPrimaryScene().addRenderableProvider(heightMapVisualizer);
            baseUI.getPrimaryScene().addRenderableProvider(zedFrameGraphic);
         }

         @Override
         public void render()
         {
            zedSVOPanel.update();
            zedPointCloudVisualizer.update();
            heightMapVisualizer.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            heightMapVisualizer.destroy();
            zedPlaybackSensor.close();
            zedFrameGraphic.dispose();

            ros2Node.close();
            baseUI.dispose();
         }
      });
   }

   private void runMethod()
   {
      try
      {
         zedPlaybackSensor.waitForGrab();
         RawImage depthImage = zedPlaybackSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
         RawImage colorImageLeft = zedPlaybackSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage colorImageRight = zedPlaybackSensor.getImage(ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);

         zedPointCloudVisualizer.setColorImage(colorImageLeft);
         zedPointCloudVisualizer.setDepthImage(depthImage);

         zedFrameGraphic.setToReferenceFrame(zedPlaybackSensor.getSensorFrame());

         RigidBodyTransformReadOnly transformToWorld = depthImage.getTransformToWorld();
         ReferenceFrame cameraFrameInWorld = new FixedReferenceFrame("FrameInWorld", ReferenceFrame.getWorldFrame(), transformToWorld);
         ZUpFrame cameraZUpFrameInWorld = new ZUpFrame(cameraFrameInWorld, "ZUpFrameInWorld");
         // Need to update this due to how its implemented, other the transform to world will be all zeros
         cameraZUpFrameInWorld.update();

         RigidBodyTransform heightMapFrameToWorldFrame = new RigidBodyTransform(depthImage.getTransformToWorld());
         Point3D heightMapCenterOrigin = new Point3D(heightMapFrameToWorldFrame.getTranslation());

         RigidBodyTransform sensorToWorld = cameraFrameInWorld.getTransformToWorldFrame();
         RigidBodyTransform sensorToGround = cameraFrameInWorld.getTransformToDesiredFrame(cameraZUpFrameInWorld);
         RigidBodyTransform groundToWorld = cameraZUpFrameInWorld.getTransformToWorldFrame();

         // Update the Z translation of the sensor to match the world transform (to handle the sensor's vertical position)
         sensorToGround.getTranslation().setZ(sensorToWorld.getTranslation().getZ());

         heightMapExtractor.update(depthImage.getGpuImageMat(),
                                   depthImage.getIntrinsicsCopy(),
                                   sensorToWorld,
                                   sensorToGround,
                                   groundToWorld,
                                   0,
                                   heightMapCenterOrigin,
                                   0);

         // Publish the height map to anyone who is subscribing
         Mat hostGlobalHeightMap = new Mat();
         // Don't destroy this mat as its being used in the extractor till that finish's
         GpuMat deviceGlobalHeightMap = heightMapExtractor.getHeightMap();
         deviceGlobalHeightMap.download(hostGlobalHeightMap);

         chunkedMapManager.update(hostGlobalHeightMap, heightMapCenterOrigin);

         hostGlobalHeightMap.close();

         publishChunkedMap();
         publishHeightMap();

         depthImage.release();
         colorImageLeft.release();
         colorImageRight.release();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   public void publishChunkedMap()
   {
      chunkedMapManager.publishChunkedMap();
   }

   public void publishHeightMap()
   {
      HeightMapMessageTools.toMessage(heightMapExtractor.getHeightMapData(), heightMapMessage);

      heightMapMessage.setSequenceId(heightMapSequenceId++);
      heightMapMessagePublisher.publish(heightMapMessage);
   }

   public static void main(String[] args)
   {
      new RDXHeightMapLogPlayer();
   }
}
