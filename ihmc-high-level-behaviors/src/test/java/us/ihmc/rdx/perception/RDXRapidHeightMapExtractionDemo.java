package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.HumanoidPerceptionModule;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXHeightMapVisualizer;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;

public class RDXRapidHeightMapExtractionDemo
{
   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("IROS_2023/20230228_200243_PerceptionLog.hdf5").toString();

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXPanel navigationPanel;

   private String sensorTopicName;

   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private Mat currentHeightMap;
   private Mat previousHeightMap;

   private ArrayList<Point2D> previousPoints = new ArrayList<>();
   private ArrayList<Point2D> currentPoints = new ArrayList<>();

   private HumanoidPerceptionModule humanoidPerception;
   private RDXHumanoidPerceptionUI humanoidPerceptionUI;
   private final RDXHeightMapVisualizer heightMapVisualizer = new RDXHeightMapVisualizer();

   private final Notification userChangedIndex = new Notification();
   private final ResettableExceptionHandlingExecutorService loadAndDecompressThreadExecutor = MissingThreadTools.newSingleThreadExecutor("LoadAndDecompress",
                                                                                                                                         true,
                                                                                                                                         1);

   private final ImInt frameIndex = new ImInt(0);
   private final ImFloat planeHeight = new ImFloat(1.5f); // 2.133f

   private final Pose3D cameraPose = new Pose3D();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());

   private final Notification heightMapUpdateNotification = new Notification();

   private final BytePointer depthBytePointer = new BytePointer(1000000);

   private final RigidBodyTransform sensorToWorldTf = new RigidBodyTransform();
   private final RigidBodyTransform sensorToGroundTf = new RigidBodyTransform();
   private final RigidBodyTransform groundToWorldTf = new RigidBodyTransform();

   private CameraIntrinsics cameraIntrinsics;

   private OpenCLManager openCLManager;
   private PerceptionDataLoader perceptionDataLoader;

   private int skipIndex = 0;
   private boolean autoIncrement = false;

   private boolean initialized = false;

   public RDXRapidHeightMapExtractionDemo()
   {
      perceptionDataLoader = new PerceptionDataLoader();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "simulation_ui");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            openCLManager = new OpenCLManager();

            navigationPanel = new RDXPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);
            heightMapVisualizer.setActive(false);

            humanoidPerception = new HumanoidPerceptionModule(openCLManager);
            humanoidPerceptionUI = new RDXHumanoidPerceptionUI(humanoidPerception, ros2Helper);

            //            createForSpherical(128, 2048);
            createForPerspective(720, 1280);

            baseUI.getPrimaryScene().addRenderableProvider(humanoidPerceptionUI.getHeightMapRenderer(), RDXSceneLevel.MODEL);
            baseUI.getImGuiPanelManager().addPanel(humanoidPerceptionUI.getRemotePerceptionUI().getPanel());
            baseUI.getPrimaryScene().addRenderableProvider(heightMapVisualizer);

            updateHeightMap();

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
            humanoidPerception.initializePerspectiveRapidHeightMapExtractor(cameraIntrinsics);
            humanoidPerceptionUI.initializeHeightMapRenderer(humanoidPerception);

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, sensorPositionBuffer, 10);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, sensorOrientationBuffer, 10);
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.L515_DEPTH_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat());

            currentHeightMap = new Mat(humanoidPerception.getRapidHeightMapExtractor().getLocalCellsPerAxis(),
                                       humanoidPerception.getRapidHeightMapExtractor().getLocalCellsPerAxis(), opencv_core.CV_8UC1);
         }

         @Override
         public void render()
         {
            if (userChangedIndex.poll())
            {
               loadAndExecute();
            }

            if (skipIndex % 30 == 0 && autoIncrement)
            {
               frameIndex.set(frameIndex.get() + 1);
               loadAndExecute();
            }

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
            PerceptionDebugTools.displayDepth("Depth", humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat(), 1);
            updateHeightMap();
         }

         private void renderNavigationPanel()
         {
            boolean changed = ImGui.sliderInt("Frame Index", frameIndex.getData(), 0, perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1);

            changed |= ImGui.sliderFloat("Plane Height", planeHeight.getData(), -3.0f, 3.0f);

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
      if (!humanoidPerception.getRapidHeightMapExtractor().isProcessing())
      {
         ThreadTools.startAsDaemon(() ->
                                   {
                                      LogTools.info("Update Height Map: " + frameIndex.get());

                                      Point3D position = sensorPositionBuffer.get(frameIndex.get());
                                      Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());
                                      cameraPose.set(position, orientation);
                                      cameraFrame.setPoseAndUpdate(cameraPose);

                                      long begin = System.nanoTime();

                                      sensorToWorldTf.set(sensorOrientationBuffer.get(frameIndex.get()), sensorPositionBuffer.get(frameIndex.get()));

                                      sensorToGroundTf.set(sensorToWorldTf);
                                      sensorToGroundTf.getTranslation().setX(0.0f);
                                      sensorToGroundTf.getTranslation().setY(0.0f);
                                      sensorToGroundTf.getRotation()
                                                      .set(new Quaternion(0.0f,
                                                                          sensorToGroundTf.getRotation().getPitch(),
                                                                          sensorToGroundTf.getRotation().getRoll()));

                                      humanoidPerception.getRapidHeightMapExtractor().update(sensorToWorldTf, sensorToGroundTf, groundToWorldTf);
                                      heightMapUpdateNotification.set();

                                      long end = System.nanoTime();
                                      LogTools.info("Update Height Map: {} ms", (end - begin) / 1e6);
                                   }, getClass().getSimpleName() + "RapidHeightMap");
      }

      if (heightMapUpdateNotification.poll())
      {
         groundToWorldTf.set(sensorToWorldTf);
         groundToWorldTf.getRotation().setYawPitchRoll(sensorToWorldTf.getRotation().getYaw(), 0, 0);
         groundToWorldTf.getTranslation().setZ(0);

         humanoidPerceptionUI.getHeightMapRenderer()
                             .update(groundToWorldTf,
                                     humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getPointerForAccessSpeed(),
                                     humanoidPerception.getRapidHeightMapExtractor().getCenterIndex(),
                                     humanoidPerception.getRapidHeightMapExtractor().getLocalCellSizeInMeters());

         heightMapVisualizer.acceptHeightMapMessage(HeightMapMessageTools.toMessage(humanoidPerception.getRapidHeightMapExtractor().getLatestHeightMapData()));
         heightMapVisualizer.update();

         humanoidPerception.getRapidHeightMapExtractor().setModified(false);
         humanoidPerception.getRapidHeightMapExtractor().setProcessing(false);

         PerceptionDebugTools.displayHeightMap("Local Height Map",
                                               humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getBytedecoOpenCVMat(),
                                               1,
                                               1 / (0.3f + 0.20f * humanoidPerception.getRapidHeightMapExtractor().getLocalCellSizeInMeters()));

         PerceptionDebugTools.displayHeightMap("Global Height Map",
                                               humanoidPerception.getRapidHeightMapExtractor().getGlobalHeightMapImage().getBytedecoOpenCVMat(),
                                               1,
                                               1 / (0.3f + 0.20f * humanoidPerception.getRapidHeightMapExtractor().getGlobalCellSizeInMeters()));
      }
   }

   public Point2D sphericalProject(Point3D cellCenter, int INPUT_HEIGHT, int INPUT_WIDTH)
   {
      Point2D proj = new Point2D();

      int count = 0;
      double pitchUnit = Math.PI / (2 * INPUT_HEIGHT);
      double yawUnit = 2 * Math.PI / (INPUT_WIDTH);

      int pitchOffset = INPUT_HEIGHT / 2;
      int yawOffset = INPUT_WIDTH / 2;

      double x = cellCenter.getX();
      double y = cellCenter.getY();
      double z = cellCenter.getZ();

      double radius = Math.sqrt(x * x + y * y);

      double pitch = Math.atan2(z, radius);
      int pitchCount = (pitchOffset) - (int) (pitch / pitchUnit);

      double yaw = Math.atan2(-y, x);
      int yawCount = (yawOffset) + (int) (yaw / yawUnit);

      proj.setX(pitchCount);
      proj.setY(yawCount);

      LogTools.info(String.format("Projection: [%.2f,%.2f] (Yc:%d,Pc:%d, Z:%.2f,R:%.2f)\n", yaw, pitch, yawCount, pitchCount, z, radius));

      return proj;
   }

   public void testProjection(Mat depth)
   {
      double radius = 4.0f;
      double height = 2.0f;
      double yawUnit = 2 * Math.PI / depth.cols();
      double pitchUnit = Math.PI / (2 * depth.rows());

      for (int i = 0; i < depth.cols(); i++)
      {
         Point3D point = new Point3D(radius * Math.cos(i * yawUnit), radius * Math.sin(i * yawUnit), -height);

         Point2D projection = sphericalProject(point, depth.rows(), depth.cols());

         LogTools.info("[" + i + "] Point : " + String.format("%.2f, %.2f, %.2f", point.getX(), point.getY(), point.getZ()) + " Projection : " + String.format(
               "%d, %d",
               (int) projection.getX(),
               (int) projection.getY()));
      }
   }

   public static void main(String[] args)
   {
      new RDXRapidHeightMapExtractionDemo();
   }
}
