package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.rapidRegions.ProjectionModel;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXLineGraphic;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.io.File;
import java.util.ArrayList;

public class RDXRapidRegionsExtractionDemo implements RenderableProvider
{

   private final ResettableExceptionHandlingExecutorService loadAndDecompressThreadExecutor = MissingThreadTools.newSingleThreadExecutor("LoadAndDecompress",
                                                                                                                                         true,
                                                                                                                                         1);
   private final File heightMapDirectory = new File(IHMCCommonPaths.USER_HOME_DIRECTORY.resolve("Downloads/HeightMap_Datasets/").toString());
   private final File[] files = heightMapDirectory.listFiles(name -> name.toString().contains(".hdf5"));

   private final String logFileName = files[0].toString();
   //private final String logFileName = "20230117_161540_GoodPerceptionLog.hdf5"; // Ouster
   //private final String logFileName = "IROS_2023/20230228_201947_PerceptionLog.hdf5"; // D455

   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(logFileName).toString();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());
   private final TypedNotification<PlanarRegionsList> planarRegionsListToRenderNotification = new TypedNotification<>();
   private final RDXLineGraphic rootJointGraphic = new RDXLineGraphic(0.02f, Color.RED);
   private final RDXLineGraphic mocapGraphic = new RDXLineGraphic(0.02f, Color.YELLOW);
   private final RDXRapidRegionsUI rapidRegionsUIPanel = new RDXRapidRegionsUI();
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final FramePlanarRegionsList frameRegions = new FramePlanarRegionsList();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> mocapOrientationBuffer = new ArrayList<>();
   private final BytePointer depthBytePointer = new BytePointer(1000000);
   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Point3D> mocapPositionBuffer = new ArrayList<>();
   private final Notification userChangedIndex = new Notification();
   private final ImInt frameIndex = new ImInt(0);
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final Pose3D cameraPose = new Pose3D();

   private Mat depthFloatImage;
   private Mat depthFloatImageFiltered;
   private BytedecoImage bytedecoDepthImage;
   private RDXPanel navigationPanel;
   private String sensorTopicName;

   private PerceptionDataLoader perceptionDataLoader;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;

   private int totalFrameCount = 0;

   /**
    * Class for extraction of planar regions from HDF5 perception logs with depth sensor data in the following configurations
    * <p>
    * D455 Depth Intrinsics: 392.57, 392.57, 326.69, 241.13 (640 x 480 @ 60)
    * D455 Color Intrinsics: 387.42, 386.89, 321.83, 240.61 (640 x 480 @ 60)
    * D455 Depth Intrinsics: 654.29, 654.29, 651.14, 361.89 (1280 x 720 @ 30)
    * D455 Color Intrinsics: 645.69, 644.81, 643.05, 361.02 (1280 x 720 @ 30)
    * L515 Depth Intrinsics: 730.7891, 731.0859, 528.6094, 408.1602
    */
   public RDXRapidRegionsExtractionDemo()
   {
      perceptionDataLoader = new PerceptionDataLoader();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            openCLManager = new OpenCLManager();
            openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

            navigationPanel = new RDXPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            //createForPerspective(720, 1280, false); // Real D455
            //createForPerspective(768, 1024, false); // Real L515
            //createForPerspective(768, 1280, true); // Simulated L515
            //createForSpherical(128, 2048); // Ouster OS0-128 (2048)
            //createForSpherical(64, 2048); // Ouster OS0-128 (2048)
            createForOrthographic(201, 201); // for extracting rapid planar regions from height maps

            setupTrajectoryVisualizers();
            baseUI.getPrimaryScene().addRenderableProvider(RDXRapidRegionsExtractionDemo.this, RDXSceneLevel.VIRTUAL);
            navigationPanel.setRenderMethod(this::renderNavigationPanel);
         }

         private void createForSpherical(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);
            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
            depthFloatImage = new Mat(depthHeight, depthWidth, opencv_core.CV_32FC1);
            depthFloatImageFiltered = new Mat(depthHeight, depthWidth, opencv_core.CV_32FC1);
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     bytedecoDepthImage.getBytedecoOpenCVMat());
            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, sensorPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, sensorOrientationBuffer);
            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, mocapPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, mocapOrientationBuffer);

            pointCloudRenderer.create(depthHeight * depthWidth);
            rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager, openCLProgram, depthHeight, depthWidth, ProjectionModel.SPHERICAL);
            rapidPlanarRegionsExtractor.getDebugger().setEnabled(true);
            rapidPlanarRegionsExtractor.getDebugger().setShowPointCloud(false);

            rapidRegionsUIPanel.create(rapidPlanarRegionsExtractor);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());

            totalFrameCount = (int) (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1);
         }

         private void createForPerspective(int depthHeight, int depthWidth, boolean simulation)
         {
            sensorTopicName = PerceptionLoggerConstants.L515_DEPTH_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);
            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.L515_DEPTH_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     bytedecoDepthImage.getBytedecoOpenCVMat());
            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.DEPTH_SENSOR_POSITION, sensorPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.DEPTH_SENSOR_ORIENTATION, sensorOrientationBuffer);
            //            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, mocapPositionBuffer);
            //            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, mocapOrientationBuffer);

            String version = simulation ? "Simulation" : "";
            rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager,
                                                                          openCLProgram,
                                                                          depthHeight,
                                                                          depthWidth,
                                                                          654.29,
                                                                          654.29,
                                                                          651.14,
                                                                          361.89,
                                                                          version);
            rapidPlanarRegionsExtractor.getDebugger().setEnabled(true);

            pointCloudRenderer.create(depthHeight * depthWidth);
            rapidRegionsUIPanel.create(rapidPlanarRegionsExtractor);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());

            totalFrameCount = (int) (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1);
         }

         private void createForOrthographic(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);
            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     bytedecoDepthImage.getBytedecoOpenCVMat());
            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.DEPTH_SENSOR_POSITION, sensorPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.DEPTH_SENSOR_ORIENTATION, sensorOrientationBuffer);
            rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager, openCLProgram, depthHeight, depthWidth, ProjectionModel.ORTHOGRAPHIC);
            rapidPlanarRegionsExtractor.getDebugger().setEnabled(true);

            rapidRegionsUIPanel.create(rapidPlanarRegionsExtractor);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());
            totalFrameCount = (int) (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1);
         }

         public void setupTrajectoryVisualizers()
         {
            //            if (!mocapPositionBuffer.isEmpty())
            //            {
            //               MocapTools.adjustMocapPositionsByOffset(mocapPositionBuffer, sensorPositionBuffer.get(0));
            //
            //               mocapGraphic.generateMeshes(mocapPositionBuffer, 10);
            //               mocapGraphic.update();
            //
            //               baseUI.getPrimaryScene().addRenderableProvider(mocapGraphic, RDXSceneLevel.VIRTUAL);
            //            }

            //            baseUI.getPrimaryScene().addRenderableProvider(rootJointGraphic, RDXSceneLevel.VIRTUAL);

            rootJointGraphic.generateMeshes(sensorPositionBuffer, 5);
            rootJointGraphic.update();
         }

         @Override
         public void render()
         {

            if (userChangedIndex.poll())
            {
               loadAndDecompressThreadExecutor.clearQueueAndExecute(() ->
                                                                    {
                                                                       perceptionDataLoader.loadCompressedDepth(sensorTopicName,
                                                                                                                frameIndex.get(),
                                                                                                                depthBytePointer,
                                                                                                                bytedecoDepthImage.getBytedecoOpenCVMat());
                                                                       ThreadTools.sleep(100);
                                                                    });
            }

            updateRapidRegionsExtractor();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderNavigationPanel()
         {
            boolean changed = ImGui.sliderInt("Frame Index", frameIndex.getData(), 0, totalFrameCount);

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
               userChangedIndex.set();
         }

         @Override
         public void dispose()
         {
            rapidPlanarRegionsExtractor.setProcessing(false);
            perceptionDataLoader.closeLogFile();
            rapidRegionsUIPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   public void updatePointCloudRenderer()
   {
      if (rapidRegionsUIPanel.getPointCloudRenderEnabled())
      {
         pointCloudRenderer.setPointsToRender(rapidPlanarRegionsExtractor.getDebugger().getDebugPoints(), Color.GRAY);
         pointCloudRenderer.updateMesh();
      }
   }

   private void updateRapidRegionsExtractor()
   {
      if (!rapidPlanarRegionsExtractor.isProcessing())
      {
         rapidPlanarRegionsExtractor.getDebugger().setShowPointCloud(rapidRegionsUIPanel.getPointCloudRenderEnabled());
         ThreadTools.startAsDaemon(() ->
                                   {
                                      Point3D position = sensorPositionBuffer.get(frameIndex.get());
                                      Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());

                                      // sensorTransformToWorld.set(orientation, position);
                                      cameraPose.set(position, orientation);
                                      cameraFrame.setPoseAndUpdate(cameraPose);

                                      cameraFrame.update();

                                      synchronized (bytedecoDepthImage.getBytedecoOpenCVMat())
                                      {
                                         frameRegions.getPlanarRegionsList().clear();

                                         // applyBilateralFilter();

                                         rapidPlanarRegionsExtractor.update(bytedecoDepthImage, cameraFrame, frameRegions);
                                         frameRegions.getPlanarRegionsList().applyTransform(cameraFrame.getTransformToWorldFrame());
                                         planarRegionsListToRenderNotification.set(frameRegions.getPlanarRegionsList().copy());
                                      }
                                   }, getClass().getSimpleName() + "RapidRegions");
      }

      if (planarRegionsListToRenderNotification.poll())
      {
         generateMesh(planarRegionsListToRenderNotification.read());
         updatePointCloudRenderer();
      }
   }

   public void applyBilateralFilter()
   {
      // perform bilateral filtering on the depth image using opencv_imgproc.bilateralFilter
      bytedecoDepthImage.getBytedecoOpenCVMat().convertTo(depthFloatImage, opencv_core.CV_32FC1, 1000.0f, 0.0);
      opencv_imgproc.bilateralFilter(depthFloatImage, depthFloatImageFiltered, 5, 50, 50);
      depthFloatImageFiltered.convertTo(bytedecoDepthImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1, 1 / 1000.0f, 0.0);
   }

   public synchronized void generateMesh(PlanarRegionsList regionsList)
   {
      if (rapidPlanarRegionsExtractor.isModified())
      {
         FramePlanarRegionsList framePlanarRegionsList = new FramePlanarRegionsList(regionsList, cameraFrame.getTransformToWorldFrame());
         rapidRegionsUIPanel.render3DGraphics(framePlanarRegionsList);
         rapidRegionsUIPanel.render();
         rapidPlanarRegionsExtractor.setModified(false);
         rapidPlanarRegionsExtractor.setProcessing(false);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (rapidRegionsUIPanel.getPointCloudRenderEnabled())
         pointCloudRenderer.getRenderables(renderables, pool);

      if (rapidRegionsUIPanel.get3DPlanarRegionsRenderEnabled())
         rapidRegionsUIPanel.getRenderables(renderables, pool);
   }

   public static void main(String[] args)
   {
      new RDXRapidRegionsExtractionDemo();
   }
}