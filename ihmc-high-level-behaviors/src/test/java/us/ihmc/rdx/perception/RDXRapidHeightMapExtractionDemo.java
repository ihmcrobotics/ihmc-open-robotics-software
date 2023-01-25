package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.logging.HDF5Manager;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXHeightMapRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.thread.Activator;

import java.util.ArrayList;

public class RDXRapidHeightMapExtractionDemo
{

   String PERCEPTION_LOG_FILE = "20230117_162825_PerceptionLog.hdf5";
   String PERCEPTION_LOG_DIRECTORY = System.getProperty("user.home") + "/.ihmc/logs/perception/";

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private ImGuiPanel navigationPanel;

   private String sensorTopicName;

   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private final RapidHeightMapExtractor rapidHeightMapUpdater = new RapidHeightMapExtractor();
   private final RDXHeightMapRenderer heightMapRenderer = new RDXHeightMapRenderer();
   ;
   private ImInt frameIndex = new ImInt(0);
   private final Pose3D cameraPose = new Pose3D();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());

   private Activator nativesLoadedActivator;
   private BytedecoImage loadedDepthImage;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private PerceptionDataLoader perceptionDataLoader;

   private boolean initialized = false;

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
            openCLManager.create();
            openCLProgram = openCLManager.loadProgram("RapidHeightMapExtractor");

            navigationPanel = new ImGuiPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            createForOuster(128, 2048);

            updateHeightMap();
         }

         private void createForOuster(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
            perceptionDataLoader.openLogFile(PERCEPTION_LOG_DIRECTORY + PERCEPTION_LOG_FILE);

            loadedDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, sensorPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, sensorOrientationBuffer);

            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, frameIndex.get(), loadedDepthImage.getBytedecoOpenCVMat());
            loadedDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

            rapidHeightMapUpdater.create(openCLManager, openCLProgram, loadedDepthImage);
            heightMapRenderer.create(rapidHeightMapUpdater.getGridLength() * rapidHeightMapUpdater.getGridWidth());
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

               if (initialized)
               {
                  // For real-time update
//                  updateHeightMap();
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
                                              (int) (perceptionDataLoader.getHDF5Manager().getCount(sensorTopicName) - 1));

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

            if (changed || !initialized)
            {
               if ((frameIndex.get() % HDF5Manager.MAX_BUFFER_SIZE) != (HDF5Manager.MAX_BUFFER_SIZE - 1))
               {
                  LogTools.info("Loading sensor data: " + frameIndex.get());
                  perceptionDataLoader.loadCompressedDepth(sensorTopicName, frameIndex.get(), loadedDepthImage.getBytedecoOpenCVMat());
                  updateHeightMap();
               }

               if (!initialized)
               {
                  initialized = true;
               }
            }
         }

         @Override
         public void dispose()
         {
            rapidHeightMapUpdater.setProcessing(false);
            perceptionDataLoader.destroy();
            openCLManager.destroy();
            baseUI.dispose();
         }
      });
   }

   private void updateHeightMap()
   {
      if (!rapidHeightMapUpdater.isProcessing())
      {
         if ((frameIndex.get() % HDF5Manager.MAX_BUFFER_SIZE) != (HDF5Manager.MAX_BUFFER_SIZE - 1))
         {
            LogTools.info("Update Height Map: " + frameIndex.get());
            ThreadTools.startAsDaemon(() ->
                                      {
                                         Point3D position = sensorPositionBuffer.get(frameIndex.get());
                                         Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());
                                         cameraPose.set(position, orientation);
                                         cameraFrame.setPoseAndUpdate(cameraPose);

                                         rapidHeightMapUpdater.update();

                                      }, getClass().getSimpleName() + "RapidRegions");
         }
      }

      //heightMapRenderer.update();
      BytedecoOpenCVTools.displayDepth("Output Height Map", rapidHeightMapUpdater.getOutputHeightMapImage().getBytedecoOpenCVMat(), 1);
      rapidHeightMapUpdater.setModified(false);
      rapidHeightMapUpdater.setProcessing(false);
   }



   public static void main(String[] args)
   {
      new RDXRapidHeightMapExtractionDemo();
   }
}
