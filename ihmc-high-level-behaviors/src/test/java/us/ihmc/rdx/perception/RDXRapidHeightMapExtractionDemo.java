package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXHeightMapRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
// Rapid height map extraction demo needs atleast the Ouster camera with depth/sensor data
public class RDXRapidHeightMapExtractionDemo
{
   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20230117_162417_Mocap_Ouster_L515_plogFixed.hdf5").toString(); // 20230117_161540_GoodPerceptionLog.hdf5

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ImGuiPanel navigationPanel;

   private String sensorTopicName;

   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private final RapidHeightMapExtractor rapidHeightMapUpdater = new RapidHeightMapExtractor();
   private final RDXHeightMapRenderer heightMapRenderer = new RDXHeightMapRenderer();

   private final Notification userChangedIndex = new Notification();
   private final ResettableExceptionHandlingExecutorService loadAndDecompressThreadExecutor = MissingThreadTools.newSingleThreadExecutor("LoadAndDecompress",
                                                                                                                                         true,
                                                                                                                                         1);

   private ImInt frameIndex = new ImInt(0);
   private ImFloat planeHeight = new ImFloat(1.5f); // 2.133f

   private final ImBoolean autoAdvance = new ImBoolean(false);

   private final Pose3D cameraPose = new Pose3D();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());

   private final Notification heightMapUpdateNotification = new Notification();

   private BytedecoImage loadedDepthImage;
   private final BytePointer depthBytePointer = new BytePointer(1000000); // holds the depth bytes in the current frame

   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private PerceptionDataLoader perceptionDataLoader;

   private boolean initialized = false;

   public RDXRapidHeightMapExtractionDemo() // Constructor
   {
      perceptionDataLoader = new PerceptionDataLoader(); // used to decode ouster depth data

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter() //ApplicationAdapter implements create(), createForOuster(), render(), renderNavigationPanel(), and dispose()
      {
         @Override
         public void create()
         {
            baseUI.create();

            openCLManager = new OpenCLManager();
            openCLProgram = openCLManager.loadProgram("RapidHeightMapExtractor"); // load the kernel program

            navigationPanel = new ImGuiPanel("Dataset Navigation Panel"); // shows parameters/ variables
            baseUI.getImGuiPanelManager().addPanel(navigationPanel); // add panel to window

            baseUI.getPrimaryScene().addRenderableProvider(heightMapRenderer, RDXSceneLevel.VIRTUAL); // heightMapRenderer displays the 3D stuff in the Primary Scene

            createForOuster(128, 2048); // frame size of ouster

            updateHeightMap(); // update height map image based on calculations from the log data. This just loads the starting frame

            // testProjection(loadedDepthImage.getBytedecoOpenCVMat());

            navigationPanel.setRenderMethod(this::renderNavigationPanel); // navigation panel seems to runs on its own thread
         }

         private void createForOuster(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);

            loadedDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1); // object to store each frame pulled from the plogs

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, sensorPositionBuffer); // tracks the ouster position in each frame
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, sensorOrientationBuffer); // tracks ouster orientation

            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, // load the first depth frame
                                                     frameIndex.get(), // initialized to 0
                                                     depthBytePointer, // save bytes to this pointer
                                                     loadedDepthImage.getBytedecoOpenCVMat()); // then compress that pointer into PNG and save as matrix in loadedDepthImage object
            loadedDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY); // *************************

            rapidHeightMapUpdater.create(openCLManager, openCLProgram, loadedDepthImage); // object that manages bulk of calculation (only has create() and update() functions)
            heightMapRenderer.create(rapidHeightMapUpdater.getGridLength() * rapidHeightMapUpdater.getGridWidth()); // object that displays height map
         }

         @Override
         public void render() // doesn't seem to be called
         {
            if (autoAdvance.get())
            {
               frameIndex.set(frameIndex.get() + 1);
               userChangedIndex.set();
               // if frame index is unsynced with height map rendering, stop auto advancing (I think)
               if (frameIndex.get() == (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1))
                  autoAdvance.set(false);
            }

            if (userChangedIndex.poll()) // when user interacts with panel
            {
               // load the depth data using the new frame the user indexed to
               loadAndDecompressThreadExecutor.clearQueueAndExecute(() -> perceptionDataLoader.loadCompressedDepth(sensorTopicName, // OUSTER depth data
                                                                                                                   frameIndex.get(), // frame to use
                                                                                                                   depthBytePointer, // pointer to load initial frame byte data to
                                                                                                                   loadedDepthImage.getBytedecoOpenCVMat())); // decoded and compressed PNG depth image stored in a matrix
               updateHeightMap(); // update height map now that the new data is loaded
            }

            baseUI.renderBeforeOnScreenUI(); // load panel window
            baseUI.renderEnd();
         }

         private void renderNavigationPanel() // layout of buttons and actions to perform when the panel is interacted with
         {
            // checks if the slider is moved
            boolean changed = ImGui.sliderInt("Frame Index",
                                              frameIndex.getData(),
                                              0, perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1);
            // slider could be a float ..?
            changed |= ImGui.sliderFloat("Plane Height", planeHeight.getData(), -3.0f, 3.0f);

            if (ImGui.button("AutoAdvance"))
               autoAdvance.set(true);
            if (ImGui.button("Stop Advancing"))
               autoAdvance.set(false);

            if (ImGui.button("Load Previous")) // if load previous is pressed, decriment frame index
            {
               frameIndex.set(Math.max(0, frameIndex.get() - 1));
               changed = true;
            }
            ImGui.sameLine(); // put prev/ next buttons next to each other
            if (ImGui.button("Load Next")) // increment index
            {
               frameIndex.set(frameIndex.get() + 1);
               changed = true;
            }

            if (changed)
            {
               userChangedIndex.set(); // if index was changed, signal to the thread that calculates height map
            }

         }

         @Override
         public void dispose() // dump threads and resources when exiting program (might need more dumping)
         {
            rapidHeightMapUpdater.setProcessing(false);
            perceptionDataLoader.closeLogFile();
            openCLManager.destroy();
            heightMapRenderer.dispose();
            baseUI.dispose();
         }
      });
   }

   private void updateHeightMap() // First function in RDXRapidHeightMapExtractionDemo class
   {
      if (!rapidHeightMapUpdater.isProcessing())
      {
         ThreadTools.startAsDaemon(() -> // seperate thread that updates height map when processing is done (Daemon basically means it's not as important)
         {
            LogTools.info("Update Height Map: " + frameIndex.get());
            Point3D position = sensorPositionBuffer.get(frameIndex.get()); // get position (of camera ..? is this in reference to world frame?)
            Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get()); // get orientation
            cameraPose.set(position, orientation);
            cameraFrame.setPoseAndUpdate(cameraPose); // camera has a specific position/rotation that it's in

            long begin = System.nanoTime();

            RigidBodyTransform transform = new RigidBodyTransform(sensorOrientationBuffer.get(frameIndex.get()), // get the transform based on the new camera position/ orientation
                                                                  sensorPositionBuffer.get(frameIndex.get()));

            // Point3D euler = new Point3D();
            // sensorOrientationBuffer.get(frameIndex.get()).getEuler(euler);
            // RigidBodyTransform transform = new RigidBodyTransform(new Quaternion(0.0, pitchAngle.get(), 0.0), new Point3D(0.0,0.0,1.0));

            // LogTools.info("Rotation: " + euler);

            rapidHeightMapUpdater.update(transform, planeHeight.get()); // update the height map by applying the transform found using the ouster position/ orientation
            heightMapUpdateNotification.set(); // send a msg from this thread to main thread that height map processing is done?

            long end = System.nanoTime();
            LogTools.info("Update Height Map: {} ms", (end - begin) / 1e6); // how long it took to update height map
         }, getClass().getSimpleName() + "RapidHeightMap");
      }

      if (heightMapUpdateNotification.poll()) // main thread: when notification occurs that means the 3D sim can be updated
      {
         // update the 3D sim using the new height map and other reference parameters
         heightMapRenderer.update(rapidHeightMapUpdater.getOutputHeightMapImage().getPointerForAccessSpeed(),
                                  rapidHeightMapUpdater.getGridLength(),
                                  rapidHeightMapUpdater.getGridWidth(),
                                  rapidHeightMapUpdater.getCellSizeXYInMeters());

         rapidHeightMapUpdater.setModified(false);
         rapidHeightMapUpdater.setProcessing(false);

         PerceptionDebugTools.displayHeightMap("Output Height Map",
                                               rapidHeightMapUpdater.getOutputHeightMapImage().getBytedecoOpenCVMat(),
                                               1,
                                               1 / (0.3f + 0.20f * rapidHeightMapUpdater.getCellSizeXYInMeters()));
      }
   }

   public Point2D sphericalProject(Point3D cellCenter, int INPUT_HEIGHT, int INPUT_WIDTH) // this for spherical/curved planar regions?
   {
      Point2D proj = new Point2D();

      // int count = 0; UNUSED ?
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

   public void testProjection(Mat depth) // unused test function
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
