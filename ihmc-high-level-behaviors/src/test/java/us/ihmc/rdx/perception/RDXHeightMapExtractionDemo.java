package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.depthData.PointCloudData;
import us.ihmc.perception.heightMap.HeightMapInputData;
import us.ihmc.perception.heightMap.HeightMapUpdater;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionDataTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXHeightMapVisualizer;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.FloatBuffer;
import java.time.Instant;
import java.util.ArrayList;

public class RDXHeightMapExtractionDemo
{
   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20230117_161540_GoodPerceptionLog.hdf5").toString();

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXPanel navigationPanel;

   private String sensorTopicName;

   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private final HeightMapUpdater heightMapUpdater = new HeightMapUpdater();
   private final RDXHeightMapVisualizer heightMapVisualizer = new RDXHeightMapVisualizer();
   private final RDXStoredPropertySetTuner heightMapParameters;
   private final RDXStoredPropertySetTuner heightMapFitlerParameters;

   private final Notification userChangedIndex = new Notification();

   private final ResettableExceptionHandlingExecutorService loadAndDecompressThreadExecutor = MissingThreadTools.newSingleThreadExecutor("LoadAndDecompress",
                                                                                                                                         true,
                                                                                                                                         1);

   private final ImInt frameIndex = new ImInt(0);
   private final ImFloat planeHeight = new ImFloat(1.5f); // 2.133f
   private final ImBoolean autoAdvance = new ImBoolean(false);

   private final Pose3D previousPose = new Pose3D();
   private final Pose3D cameraPose = new Pose3D();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());

   private final Notification heightMapUpdateNotification = new Notification();

   private BytedecoImage loadedDepthImage;
   private final BytePointer depthBytePointer = new BytePointer(1000000);
   private double translation = Double.NaN;

   private OpenCLManager openCLManager;
   private PerceptionDataLoader perceptionDataLoader;

   private boolean initialized = false;

   public RDXHeightMapExtractionDemo()
   {
      perceptionDataLoader = new PerceptionDataLoader();
      previousPose.setToNaN();

      heightMapParameters = new RDXStoredPropertySetTuner("heightMapParameters");
      heightMapFitlerParameters = new RDXStoredPropertySetTuner("Filter Parameters");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            openCLManager = new OpenCLManager();

            heightMapVisualizer.create();
            heightMapVisualizer.setActive(true);
            heightMapUpdater.attachHeightMapConsumer(heightMapVisualizer::acceptHeightMapMessage);

            heightMapParameters.create(heightMapUpdater.getHeightMapParameters());
            heightMapFitlerParameters.create(heightMapUpdater.getHeightMapFilterParameters());

            navigationPanel = new RDXPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);
            baseUI.getImGuiPanelManager().addPanel(heightMapParameters);
            baseUI.getImGuiPanelManager().addPanel(heightMapFitlerParameters);

            baseUI.getPrimaryScene().addRenderableProvider(heightMapVisualizer);

            createForOuster(128, 2048);

            updateHeightMap();

            // testProjection(loadedDepthImage.getBytedecoOpenCVMat());

            navigationPanel.setRenderMethod(this::renderNavigationPanel);
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
         }

         @Override
         public void render()
         {
            if (autoAdvance.get())
            {
               frameIndex.set(frameIndex.get() + 1);
               userChangedIndex.set();
               if (frameIndex.get() == (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1))
                  autoAdvance.set(false);
            }

            if (userChangedIndex.poll())
            {
               loadAndDecompressThreadExecutor.clearQueueAndExecute(() -> perceptionDataLoader.loadCompressedDepth(sensorTopicName,
                                                                                                                   frameIndex.get(),
                                                                                                                   depthBytePointer,
                                                                                                                   loadedDepthImage.getBytedecoOpenCVMat()));
               updateHeightMap();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderNavigationPanel()
         {
            boolean changed = ImGui.sliderInt("Frame Index", frameIndex.getData(), 0, (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1));

            changed |= ImGui.sliderFloat("Plane Height", planeHeight.getData(), -3.0f, 3.0f);
            if (ImGui.button("AutoAdvance"))
               autoAdvance.set(true);
            if (ImGui.button("Stop Advancing"))
               autoAdvance.set(false);

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

            if (changed)
            {
               userChangedIndex.set();
            }
            imgui.internal.ImGui.text("Distance " + translation);

            ImGui.separator();

            heightMapVisualizer.renderImGuiWidgets();
         }

         @Override
         public void dispose()
         {
            perceptionDataLoader.closeLogFile();
            openCLManager.destroy();
            baseUI.dispose();
         }
      });
   }

   public void updateWithDataBuffer(ReferenceFrame groundFrame,
                                    ReferenceFrame sensorFrame,
                                    BytedecoImage depthImage,
                                    int numberOfPoints,
                                    Instant instant,
                                    boolean isMoving)
   {

      FramePose3D sensorPose = new FramePose3D(sensorFrame);
      sensorPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D gridCenter = new Point3D(sensorPose.getX(), sensorPose.getY(), 0.0);
      FloatBuffer pointCloudInSensorFrame = PerceptionDataTools.convertSphericalDepthImageToPointCloudInSensorFrame(depthImage, Math.PI / 2.0, 2.0 * Math.PI);
      PointCloudData pointCloudData = new PointCloudData(instant, numberOfPoints, pointCloudInSensorFrame);
      HeightMapInputData inputData = new HeightMapInputData();
      inputData.pointCloud = pointCloudData;
      inputData.gridCenter = gridCenter;
      // submitting the world frame for the sensor pose, as that's the frame the data is in.
      inputData.sensorPose = sensorPose;
      // TODO add variance
      if (isMoving)
      {
         inputData.verticalMeasurementVariance = heightMapUpdater.getHeightMapParameters().getSensorVarianceWhenMoving();
      }
      else
      {
         inputData.verticalMeasurementVariance = heightMapUpdater.getHeightMapParameters().getSensorVarianceWhenStanding();
      }

      double size = isMoving ? 0.05 : 0.1;
      RDXModelInstance coordinateFrame = new RDXModelInstance(RDXModelBuilder.createCoordinateFrameInstance(size));
      LibGDXTools.toLibGDX(new RigidBodyTransform(sensorPose), coordinateFrame.transform);
      baseUI.getPrimaryScene().addRenderableProvider(coordinateFrame);

      heightMapUpdater.addPointCloudToQueue(inputData);
   }

   private void updateHeightMap()
   {
      LogTools.info("Update Height Map: " + frameIndex.get());
      Point3D position = sensorPositionBuffer.get(frameIndex.get());
      Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());
      cameraPose.set(position, orientation);
      cameraFrame.setPoseAndUpdate(cameraPose);

      boolean validTranslation = cameraPose.getTranslation().distance(previousPose.getTranslation()) < 5.0;
      boolean validRotation = cameraPose.getRotation().distance(previousPose.getOrientation()) < 0.1;
      if (previousPose.containsNaN() || validTranslation && validRotation)
      {
         translation = cameraPose.getTranslation().distance(previousPose.getTranslation());
         boolean isMoving = translation > 0.005;
         updateWithDataBuffer(ReferenceFrame.getWorldFrame(),
                              cameraFrame,
                              loadedDepthImage,
                              loadedDepthImage.getImageHeight() * loadedDepthImage.getImageWidth(),
                              Instant.now(),
                              isMoving);
         heightMapUpdater.runUpdateThread();

         heightMapVisualizer.update();
         previousPose.set(cameraPose);
      }

      //      if (heightMapUpdateNotification.poll())
      //      {
      //         heightMapRenderer.update(rapidHeightMapUpdater.getOutputHeightMapImage().getPointerForAccessSpeed(),
      //                                  rapidHeightMapUpdater.getGridLength(),
      //                                  rapidHeightMapUpdater.getGridWidth(),
      //                                  rapidHeightMapUpdater.getCellSizeXYInMeters());
      //
      //         rapidHeightMapUpdater.setModified(false);
      //         rapidHeightMapUpdater.setProcessing(false);
      //
      //         PerceptionDebugTools.displayHeightMap("Output Height Map",
      //                                               rapidHeightMapUpdater.getOutputHeightMapImage().getBytedecoOpenCVMat(),
      //                                               1,
      //                                              1 / (0.3f + 0.20f * rapidHeightMapUpdater.getCellSizeXYInMeters()));
      //      }

   }

   public static void main(String[] args)
   {
      new RDXHeightMapExtractionDemo();
   }
}
