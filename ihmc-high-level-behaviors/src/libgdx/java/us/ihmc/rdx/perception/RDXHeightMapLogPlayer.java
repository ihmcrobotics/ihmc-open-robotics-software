package us.ihmc.rdx.perception;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.gpuMapping.HeightMapExtractor;
import us.ihmc.perception.gpuMapping.HeightMapMessageTools;
import us.ihmc.perception.gpuMapping.HeightMapParameters;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2HeightMapVisualizer;
import us.ihmc.robotModels.FullRobotModelTestTools.RandomFullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

import java.util.Random;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class RDXHeightMapLogPlayer
{
   private static final String SVO_FILE = System.getProperty("user.home") + "/Downloads/heightmap_test.svo2";

   private final ROS2ZEDSVOPlaybackSensor zedPlaybackSensor;
   private final RDXZEDSVORecorderPanel zedSVOPanel;

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;

   private final RDXBaseUI baseUI;

   private final RDXRawImagePointCloudVisualizer zedPointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud", true);
   private final BlockingQueue<RawImage> rawImageCollection;
   private final RandomFullHumanoidRobotModel robotModel;

   private final HeightMapExtractor heightMapExtractor;
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();

   private final HeightMapMessage heightMapMessage;
   private long heightMapSequenceId = 0;
   private final ROS2Publisher<HeightMapMessage> heightMapMessagePublisher;

   private final RDXROS2HeightMapVisualizer heightMapVisualizer;


   public RDXHeightMapLogPlayer()
   {
      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName());
      ros2Helper = new ROS2Helper(ros2Node);

      robotModel = new RandomFullHumanoidRobotModel(new Random());

      heightMapExtractor = new HeightMapExtractor(heightMapParameters);


      baseUI = new RDXBaseUI();

      zedPlaybackSensor = new ROS2ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_X_MINI, zed.SL_DEPTH_MODE_NEURAL_LIGHT, SVO_FILE);
      zedSVOPanel = new RDXZEDSVORecorderPanel(ros2Helper);

      heightMapVisualizer = new RDXROS2HeightMapVisualizer("Height Map Visualizer");
      heightMapVisualizer.setupForImageMessage(ros2Helper);

      rawImageCollection = new LinkedBlockingQueue<>(ImageSensor.DEFAULT_IMAGE_QUEUE_CAPACITY);
      zedPlaybackSensor.registerImageQueue(rawImageCollection, ZEDImageSensor.DEPTH_IMAGE_KEY);

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
            baseUI.getImGuiPanelManager().addPanel("ZED SVO", zedSVOPanel::render);
            zedPointCloudVisualizer.create();
            baseUI.getImGuiPanelManager().addPanel("Height Map Renderer", heightMapVisualizer::renderImGuiWidgets);

            zedPointCloudVisualizer.setActive(true);
            zedPlaybackSensor.run(true);
            heightMapVisualizer.create();
            heightMapVisualizer.setActive(true);
            baseUI.getPrimaryScene().addRenderableProvider(heightMapVisualizer::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("ZED Point Cloud", zedPointCloudVisualizer::renderImGuiWidgets);
            baseUI.getPrimaryScene().addRenderableProvider(zedPointCloudVisualizer);
         }

         @Override
         public void render()
         {
            zedPointCloudVisualizer.update();
            heightMapVisualizer.update();
            zedSVOPanel.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();

            zedPlaybackSensor.close();
            ros2Node.destroy();
            heightMapVisualizer.destroy();
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

         RigidBodyTransformReadOnly transformToWorld = depthImage.getTransformToWorld();
         ReferenceFrame cameraFrameInWorld = new FixedReferenceFrame("FrameInWorld", ReferenceFrame.getWorldFrame(), transformToWorld);
         ZUpFrame cameraZUpFrameInWorld = new ZUpFrame(cameraFrameInWorld, "ZUpFrameInWorld");
         // Need to update this due to how its implemented, other the transform to world will be all zeros
         cameraZUpFrameInWorld.update();

         RigidBodyTransform heightMapFrameToWorldFrame = new RigidBodyTransform();
         Point3D heightMapCenterOrigin = new Point3D(heightMapFrameToWorldFrame.getTranslation());

         RigidBodyTransform sensorToWorld = cameraFrameInWorld.getTransformToWorldFrame();
         RigidBodyTransform sensorToGround = cameraFrameInWorld.getTransformToDesiredFrame(cameraZUpFrameInWorld);
         RigidBodyTransform groundToWorld = cameraZUpFrameInWorld.getTransformToWorldFrame();

         heightMapExtractor.update(depthImage.getGpuImageMat(),
                                   depthImage.getIntrinsicsCopy(),
                                   sensorToWorld,
                                   sensorToGround,
                                   groundToWorld,
                                   0,
                                   heightMapCenterOrigin,
                                   0);

//         Mat what  = new Mat();
//         heightMapExtractor.getHeightMap().download(what);
//         PerceptionDebugTools.printMat("s", what, 5);

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



   public void publishHeightMap()
   {
      HeightMapMessageTools.toMessage(heightMapExtractor.getHeightMapData(), heightMapMessage);

      //      heightMapLogger.logHeightMap(globalHeightMap, heightMapCenterPoint);

      heightMapMessage.setSequenceId(heightMapSequenceId++);
      heightMapMessagePublisher.publish(heightMapMessage);
   }

   public static void main(String[] args)
   {
      RDXHeightMapLogPlayer heightMapLogPlayer = new RDXHeightMapLogPlayer();
   }
}
