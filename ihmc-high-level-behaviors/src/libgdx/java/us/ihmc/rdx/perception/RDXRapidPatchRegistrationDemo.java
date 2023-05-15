package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.ihmcPerception.depthData.PointCloudData;
import us.ihmc.ihmcPerception.heightMap.HeightMapInputData;
import us.ihmc.ihmcPerception.heightMap.HeightMapUpdater;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.odometry.RapidPatchesBasedICP;
import us.ihmc.perception.rapidRegions.RapidPatchesDebugOutputGenerator;
import us.ihmc.perception.tools.PerceptionDataTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXHeightMapVisualizer;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.FloatBuffer;
import java.time.Instant;
import java.util.ArrayList;

public class RDXRapidPatchRegistrationDemo
{
   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20230117_161540_GoodPerceptionLog.hdf5").toString();

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ImGuiPanel navigationPanel;

   private String sensorTopicName;

   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

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

   private Activator nativesLoadedActivator;

   private BytedecoImage loadedDepthImage;
   private final BytePointer depthBytePointer = new BytePointer(1000000);
   private double translation = Double.NaN;

   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private PerceptionDataLoader perceptionDataLoader;

   private final RapidPatchesBasedICP rapidPatchesBasedICP = new RapidPatchesBasedICP();
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final RapidPatchesDebugOutputGenerator debugger = new RapidPatchesDebugOutputGenerator();


   private final int depthHeight = 128;
   private final int depthWidth = 1024;

   public RDXRapidPatchRegistrationDemo()
   {
      perceptionDataLoader = new PerceptionDataLoader();
      previousPose.setToNaN();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
            baseUI.create();

            openCLManager = new OpenCLManager();
            openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");
            rapidPatchesBasedICP.create(openCLManager, openCLProgram, depthHeight, depthWidth);

            navigationPanel = new ImGuiPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            createForOuster(depthHeight, depthWidth);

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

            pointCloudRenderer.create(depthHeight * depthWidth);
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

               if (autoAdvance.get())
               {
                  frameIndex.set(frameIndex.get() + 1);
                  userChangedIndex.set();
                  if (frameIndex.get() ==  (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1))
                     autoAdvance.set(false);
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
                  registerPatchCloud();
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
                                              (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1));

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

   public void updatePointCloudRenderer()
   {
      pointCloudRenderer.setPointsToRender(debugger.getDebugPoints(), Color.GRAY);
      pointCloudRenderer.updateMesh();
   }

   private void registerPatchCloud()
   {
      LogTools.info("Register Patch Cloud: " + frameIndex.get());

      Point3D position = sensorPositionBuffer.get(frameIndex.get());
      Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());
      cameraPose.set(position, orientation);
      cameraFrame.setPoseAndUpdate(cameraPose);

      boolean validTranslation = cameraPose.getTranslation().distance(previousPose.getTranslation()) < 5.0;
      boolean validRotation = cameraPose.getRotation().distance(previousPose.getOrientation()) < 0.1;
      if (previousPose.containsNaN() || validTranslation && validRotation)
      {

      }

      updatePointCloudRenderer();

   }




   public static void main(String[] args)
   {
      new RDXRapidPatchRegistrationDemo();
   }
}
