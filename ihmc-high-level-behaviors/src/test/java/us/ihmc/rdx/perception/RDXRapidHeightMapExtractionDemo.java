package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
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
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXHeightMapRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXHeightMapVisualizer;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;

public class RDXRapidHeightMapExtractionDemo
{
   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20230117_161540_GoodPerceptionLog.hdf5").toString();

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ImGuiPanel navigationPanel;

   private String sensorTopicName;

   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private final RapidHeightMapExtractor rapidHeightMapUpdater = new RapidHeightMapExtractor();
   private final RDXHeightMapRenderer heightMapRenderer = new RDXHeightMapRenderer();
   private final RDXHeightMapVisualizer heightMapVisualizer = new RDXHeightMapVisualizer();

   private final Notification userChangedIndex = new Notification();
   private final ResettableExceptionHandlingExecutorService loadAndDecompressThreadExecutor = MissingThreadTools.newSingleThreadExecutor("LoadAndDecompress",
                                                                                                                                         true,
                                                                                                                                         1);

   private final ImBoolean fastRendererEnabled = new ImBoolean(true);
   private final ImBoolean basicVisualizerEnabled = new ImBoolean(false);

   private final ImInt frameIndex = new ImInt(0);
   private final ImFloat planeHeight = new ImFloat(1.5f); // 2.133f
   private final ImFloat sensorpitch = new ImFloat(30.0f);

   private final Pose3D cameraPose = new Pose3D();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());

   private final Notification heightMapUpdateNotification = new Notification();

   private Activator nativesLoadedActivator;

   private BytedecoImage loadedDepthImage;
   private final BytePointer depthBytePointer = new BytePointer(1000000);

   private OpenCLManager openCLManager;
   private PerceptionDataLoader perceptionDataLoader;

   private boolean initialized = false;
   private int totalDepthCount = 0;

   public RDXRapidHeightMapExtractionDemo()
   {
      perceptionDataLoader = new PerceptionDataLoader();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
            baseUI.create();

            openCLManager = new OpenCLManager();

            navigationPanel = new ImGuiPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);
            heightMapVisualizer.setActive(true);

            baseUI.getPrimaryScene().addRenderableProvider(heightMapRenderer, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(heightMapVisualizer);

            createForOuster(128, 2048);

            updateHeightMap();

            // testProjection(loadedDepthImage.getBytedecoOpenCVMat());
         }

         private void createForOuster(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);

            loadedDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, sensorPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, sensorOrientationBuffer);

            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     loadedDepthImage.getBytedecoOpenCVMat());
            loadedDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

            rapidHeightMapUpdater.create(openCLManager, loadedDepthImage);
            heightMapRenderer.create(rapidHeightMapUpdater.getCellsPerAxis() * rapidHeightMapUpdater.getCellsPerAxis());

            totalDepthCount = (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  baseUI.getLayoutManager().reloadLayout();
                  navigationPanel.setRenderMethod(this::renderNavigationPanel);
               }

               if (userChangedIndex.poll())
               {
                  loadAndDecompressThreadExecutor.clearQueueAndExecute(() ->
                                                                       {
                                                                          perceptionDataLoader.loadCompressedDepth(sensorTopicName,
                                                                                                                   frameIndex.get(),
                                                                                                                   depthBytePointer,
                                                                                                                   loadedDepthImage.getBytedecoOpenCVMat());
                                                                       });
                  updateHeightMap();
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderNavigationPanel()
         {
            boolean changed = ImGui.sliderInt("Frame Index",
                                              frameIndex.getData(),
                                              0,
                                              totalDepthCount);

            changed |= ImGui.sliderFloat("Plane Height", planeHeight.getData(), -3.0f, 3.0f);
            changed |= ImGui.sliderFloat("Sensor Pitch", sensorpitch.getData(), -30.0f, 30.0f);

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

            ImGui.checkbox("Fast Renderer", fastRendererEnabled);
            ImGui.checkbox("Basic Visualizer", basicVisualizerEnabled);

            if (changed)
            {
               userChangedIndex.set();
            }
         }

         @Override
         public void dispose()
         {
            rapidHeightMapUpdater.setProcessing(false);
            perceptionDataLoader.closeLogFile();
            openCLManager.destroy();
            heightMapRenderer.dispose();
            baseUI.dispose();
         }
      });
   }

   private void updateHeightMap()
   {
      if (!rapidHeightMapUpdater.isProcessing())
      {
         ThreadTools.startAsDaemon(() ->
                                   {
                                      LogTools.info("Update Height Map: " + frameIndex.get());
                                      Point3D position = sensorPositionBuffer.get(frameIndex.get());
                                      Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());
                                      orientation.multiply(new Quaternion(0, Math.toRadians(sensorpitch.get()), 0));
                                      cameraPose.set(position, orientation);
                                      cameraFrame.setPoseAndUpdate(cameraPose);

                                      long begin = System.nanoTime();


                                      // Point3D euler = new Point3D();
                                      // sensorOrientationBuffer.get(frameIndex.get()).getEuler(euler);
                                      // RigidBodyTransform transform = new RigidBodyTransform(new Quaternion(0.0, pitchAngle.get(), 0.0), new Point3D(0.0,0.0,1.0));

                                      // LogTools.info("Rotation: " + euler);

                                      rapidHeightMapUpdater.update(new RigidBodyTransform(cameraPose), planeHeight.get());
                                      heightMapUpdateNotification.set();

                                      long end = System.nanoTime();
                                      LogTools.info("Update Height Map: {} ms", (end - begin) / 1e6);
                                   }, getClass().getSimpleName() + "RapidHeightMap");
      }

      if (heightMapUpdateNotification.poll())
      {
         Point2D gridCenter = rapidHeightMapUpdater.getLatestHeightMapData() != null ? rapidHeightMapUpdater.getLatestHeightMapData().getGridCenter() : new Point2D();

         if(fastRendererEnabled.get())
         {
            heightMapRenderer.setActive(true);
            heightMapRenderer.update(rapidHeightMapUpdater.getOutputHeightMapImage().getPointerForAccessSpeed(),
                                     gridCenter,
                                     rapidHeightMapUpdater.getCenterIndex(),
                                     rapidHeightMapUpdater.getCellSizeXYInMeters());
         }
         else
         {
            heightMapRenderer.setActive(false);
         }

         if(basicVisualizerEnabled.get())
         {
            heightMapVisualizer.setActive(true);
            heightMapVisualizer.acceptHeightMapMessage(HeightMapMessageTools.toMessage(rapidHeightMapUpdater.getLatestHeightMapData()));
            heightMapVisualizer.update();
         }
         else
         {
            heightMapVisualizer.setActive(false);
         }

         rapidHeightMapUpdater.setModified(false);
         rapidHeightMapUpdater.setProcessing(false);

         PerceptionDebugTools.displayHeightMap("Output Height Map",
                                               rapidHeightMapUpdater.getOutputHeightMapImage().getBytedecoOpenCVMat(),
                                               1,
                                              1 / (0.3f + 0.20f * rapidHeightMapUpdater.getCellSizeXYInMeters()));
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
