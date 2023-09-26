package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;

/**
 * This class loads the depth maps from a perception log and extracts the height map from it. Allows for scrubbing through the log and visualizing the height
 * map for any given frame.
 * <p>
 * To Run: Download the HDF5 file used in this demo from the Google Drive from the following location:
 * Fast Behaviors > Perception > Perception Logs > SLAM_RoughTerrain_LinearLoop > IROS_2023 > 20230228_200243_PerceptionLog.hdf5
 */
public class RDXRapidHeightMapExtractionDemo
{
   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("IROS_2023/20230228_201947_PerceptionLog.hdf5").toString();
   private final ResettableExceptionHandlingExecutorService loadAndDecompressThreadExecutor = MissingThreadTools.newSingleThreadExecutor("LoadAndDecompress",
                                                                                                                                         true,
                                                                                                                                         1);
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();
   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final Notification heightMapUpdateNotification = new Notification();
   private final RigidBodyTransform sensorToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform sensorToGroundTransform = new RigidBodyTransform();
   private final RigidBodyTransform groundToWorldTransform = new RigidBodyTransform();
   private final BytePointer depthBytePointer = new BytePointer(1000000);
   private final Notification userChangedIndex = new Notification();
   private final ImInt frameIndex = new ImInt(0);
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final Pose3D cameraPose = new Pose3D();

   private final PerceptionDataLoader perceptionDataLoader;
   private RDXHumanoidPerceptionUI humanoidPerceptionUI;
   private HumanoidPerceptionModule humanoidPerception;
   private RDXBytedecoImagePanel localHeightMapPanel;
   private RDXBytedecoImagePanel internalHeightMapPanel;
   private RDXBytedecoImagePanel croppedHeightMapPanel;
   private RDXBytedecoImagePanel depthImagePanel;
   private CameraIntrinsics cameraIntrinsics;
   private OpenCLManager openCLManager;
   private RDXPanel navigationPanel;
   private String sensorTopicName;
   private ROS2Helper ros2Helper;
   private ROS2Node ros2Node;

   private PoseReferenceFrame cameraZUpFrame = new PoseReferenceFrame("CameraZUpFrame", cameraFrame);

   private int totalCount = 0;
   private int skipIndex = 0;
   private boolean autoIncrement = false;

   public RDXRapidHeightMapExtractionDemo()
   {
      perceptionDataLoader = new PerceptionDataLoader();

      ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "simulation_ui");
      ros2Helper = new ROS2Helper(ros2Node);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            openCLManager = new OpenCLManager();

            navigationPanel = new RDXPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            humanoidPerception = new HumanoidPerceptionModule(openCLManager);
            humanoidPerceptionUI = new RDXHumanoidPerceptionUI(humanoidPerception, ros2Helper);
            humanoidPerceptionUI.initializeHeightMapVisualizer(ros2Helper);
            humanoidPerceptionUI.initializeHeightMapUI(ros2Helper);

            //            createForSpherical(128, 2048);
            createForPerspective(720, 1280);

            baseUI.getImGuiPanelManager().addPanel(humanoidPerceptionUI);
            baseUI.getPrimaryScene().addRenderableProvider(humanoidPerceptionUI.getHeightMapVisualizer());

            updateHeightMap();


            internalHeightMapPanel = new RDXBytedecoImagePanel("Internal Height Map",
                                                               humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getImageWidth(),
                                                               humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getImageHeight(),
                                                               RDXImagePanel.FLIP_Y);
            depthImagePanel = new RDXBytedecoImagePanel("Depth Image",
                                                        humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat().cols(),
                                                         humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat().rows(),
                                                         RDXImagePanel.FLIP_Y);
            localHeightMapPanel = new RDXBytedecoImagePanel("Local Height Map",
                                                            humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getImageWidth(),
                                                            humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getImageHeight(),
                                                            RDXImagePanel.FLIP_Y);
            croppedHeightMapPanel = new RDXBytedecoImagePanel("Cropped Height Map",
                                                              humanoidPerception.getRapidHeightMapExtractor().getCroppedGlobalHeightMapImage().cols(),
                                                              humanoidPerception.getRapidHeightMapExtractor().getCroppedGlobalHeightMapImage().rows(),
                                                              RDXImagePanel.FLIP_Y);



            navigationPanel.addChild(localHeightMapPanel.getImagePanel());
            navigationPanel.addChild(croppedHeightMapPanel.getImagePanel());
            navigationPanel.addChild(depthImagePanel.getImagePanel());
            navigationPanel.addChild(internalHeightMapPanel.getImagePanel());

            navigationPanel.setRenderMethod(this::renderNavigationPanel);
         }

         private void createForSpherical(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, sensorPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, sensorOrientationBuffer);

            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat());
         }

         private void createForPerspective(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.L515_DEPTH_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);

            cameraIntrinsics = new CameraIntrinsics(depthHeight, depthWidth, 654.29, 654.29, 651.14, 361.89);
            humanoidPerception.initializeRealsenseDepthImage(depthHeight, depthWidth);
            humanoidPerception.initializeHeightMapExtractor(cameraIntrinsics);

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, sensorPositionBuffer, 10);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, sensorOrientationBuffer, 10);
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.L515_DEPTH_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat());

            if (totalCount == 0)
            {
               totalCount = perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1;
            }
         }

         @Override
         public void render()
         {
            if (userChangedIndex.poll())
            {
               loadAndExecute();
            }

            if (skipIndex % 30 == 0 && autoIncrement && frameIndex.get() < totalCount - 10)
            {
               frameIndex.set(frameIndex.get() + 1);
               loadAndExecute();
            }

            depthImagePanel.drawDepthImage(humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat());
            localHeightMapPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getBytedecoOpenCVMat());
            croppedHeightMapPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getCroppedGlobalHeightMapImage());

            humanoidPerceptionUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();

            skipIndex++;
         }

         public void loadAndExecute()
         {
            loadAndDecompressThreadExecutor.clearQueueAndExecute(() -> perceptionDataLoader.loadCompressedDepth(sensorTopicName,
                                                                                                                frameIndex.get(),
                                                                                                                depthBytePointer,
                                                                                                                humanoidPerception.getRealsenseDepthImage()
                                                                                                                                  .getBytedecoOpenCVMat()));
            updateHeightMap();
         }

         private void renderNavigationPanel()
         {
            boolean changed = ImGui.sliderInt("Frame Index", frameIndex.getData(), 0, totalCount);

            if (ImGui.button("Load Previous"))
            {
               frameIndex.set(Math.max(0, frameIndex.get() - 1));
               changed = true;
            }
            ImGui.sameLine();
            if (ImGui.button("Load Next"))
            {
               frameIndex.set(frameIndex.get() + 1);
               changed = true;
            }

            if (ImGui.button("Auto Increment"))
            {
               autoIncrement = true;
            }

            if (ImGui.button("Pause"))
            {
               autoIncrement = false;
            }

            if (changed)
            {
               userChangedIndex.set();
            }
         }

         @Override
         public void dispose()
         {
            humanoidPerception.destroy();
            perceptionDataLoader.closeLogFile();
            openCLManager.destroy();
            humanoidPerceptionUI.destroy();
            baseUI.dispose();
         }
      });
   }

   private void updateHeightMap()
   {
      Point3D position = sensorPositionBuffer.get(frameIndex.get());
      Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());
      cameraPose.set(position, orientation);
      cameraFrame.setPoseAndUpdate(cameraPose);

      long begin = System.nanoTime();

      sensorToWorldTransform.set(sensorOrientationBuffer.get(frameIndex.get()), sensorPositionBuffer.get(frameIndex.get()));

      groundToWorldTransform.set(sensorToWorldTransform);
      groundToWorldTransform.getRotation().setYawPitchRoll(sensorToWorldTransform.getRotation().getYaw(), 0, 0);
      groundToWorldTransform.getTranslation().setZ(0);

      sensorToGroundTransform.set(sensorToWorldTransform);
      sensorToGroundTransform.getTranslation().setX(0.0f);
      sensorToGroundTransform.getTranslation().setY(0.0f);
      sensorToGroundTransform.getRotation()
                             .set(new Quaternion(0.0f, sensorToGroundTransform.getRotation().getPitch(), sensorToGroundTransform.getRotation().getRoll()));
      RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
      groundToSensorTransform.invert();
      cameraZUpFrame.getTransformToParent().set(groundToSensorTransform);

      humanoidPerception.updateTerrain(ros2Helper,
                                       humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat(),
                                       cameraFrame,
                                       cameraZUpFrame,
                                       true,
                                       false);

      long end = System.nanoTime();
      LogTools.info("Update Height Map ({}): {} ms", frameIndex.get(), (end - begin) / 1e6);
   }

   public static void main(String[] args)
   {
      new RDXRapidHeightMapExtractionDemo();
   }
}
