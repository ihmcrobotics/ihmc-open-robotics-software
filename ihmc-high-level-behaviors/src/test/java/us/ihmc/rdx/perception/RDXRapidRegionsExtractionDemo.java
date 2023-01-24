package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.logging.HDF5Manager;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.MocapTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXLineMeshModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.thread.Activator;

import java.util.ArrayList;

public class RDXRapidRegionsExtractionDemo implements RenderableProvider
{
   //20230117_161540_PerceptionLog.hdf5 (311 MB)
   //20230117_162417_PerceptionLog.hdf5 (231 MB)
   //20230117_162825_PerceptionLog.hdf5 (328 MB)

   String PERCEPTION_LOG_FILE = "20230117_162825_PerceptionLog.hdf5";
   String PERCEPTION_LOG_DIRECTORY = System.getProperty("user.home") + "/.ihmc/logs/perception/";


   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final RDXRapidRegionsUIPanel rapidRegionsUIPanel = new RDXRapidRegionsUIPanel();
   private ImGuiPanel navigationPanel;

   private final RDXLineMeshModel mocapGraphic = new RDXLineMeshModel(0.02f, Color.YELLOW);
   private final RDXLineMeshModel rootJointGraphic = new RDXLineMeshModel(0.02f, Color.RED);

   private String sensorTopicName;

   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final OpenCLFloatParameters parametersOpenCLFloatBuffer = new OpenCLFloatParameters();
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   private final RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
   private final PlanarRegionsListWithPose regionsWithPose = new PlanarRegionsListWithPose();
   ;
   //   private final ReferenceFrame cameraFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("l515ReferenceFrame",
   //                                                                                                              ReferenceFrame.getWorldFrame(),
   //                                                                                                              sensorTransformToWorld);
   private final Pose3D cameraPose = new Pose3D();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());

   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private final ArrayList<Point3D> mocapPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> mocapOrientationBuffer = new ArrayList<>();
   ;
   private Activator nativesLoadedActivator;
   private BytedecoImage bytedecoDepthImage;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private PerceptionDataLoader perceptionDataLoader;

   private ImInt frameIndex = new ImInt(0);

   private boolean initialized = false;

   public RDXRapidRegionsExtractionDemo()
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
            openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

            navigationPanel = new ImGuiPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            createL515(768, 1024);
            //createOuster(128, 2048);

            updateRapidRegionsExtractor();
            updatePointCloudRenderer();
         }

         private void createOuster(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
            perceptionDataLoader.openLogFile(PERCEPTION_LOG_DIRECTORY + PERCEPTION_LOG_FILE);
            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, frameIndex.get(), bytedecoDepthImage.getBytedecoOpenCVMat());

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, sensorPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, sensorOrientationBuffer);

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, mocapPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, mocapOrientationBuffer);

            pointCloudRenderer.create(depthHeight * depthWidth);
            rapidPlanarRegionsExtractor.create(openCLManager, openCLProgram, depthHeight, depthWidth);

            rapidRegionsUIPanel.create(rapidPlanarRegionsExtractor);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());
         }

         private void createL515(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.L515_DEPTH_NAME;
            perceptionDataLoader.openLogFile(PERCEPTION_LOG_DIRECTORY + PERCEPTION_LOG_FILE);
            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.L515_DEPTH_NAME, frameIndex.get(), bytedecoDepthImage.getBytedecoOpenCVMat());
            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, sensorPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, sensorOrientationBuffer);

            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, mocapPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, mocapOrientationBuffer);

            rapidPlanarRegionsExtractor.create(openCLManager, openCLProgram, depthHeight, depthWidth, 730.7891, 731.0859, 528.6094, 408.1602);

            pointCloudRenderer.create(depthHeight * depthWidth);
            rapidRegionsUIPanel.create(rapidPlanarRegionsExtractor);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  baseUI.getPrimaryScene().addRenderableProvider(RDXRapidRegionsExtractionDemo.this, RDXSceneLevel.VIRTUAL);

                  MocapTools.adjustMocapPositionsByOffset(mocapPositionBuffer, sensorPositionBuffer.get(0));

                  mocapGraphic.generateMeshes(mocapPositionBuffer, 10);
                  mocapGraphic.update();

                  rootJointGraphic.generateMeshes(sensorPositionBuffer, 5);
                  rootJointGraphic.update();

                  baseUI.getPrimaryScene().addRenderableProvider(mocapGraphic, RDXSceneLevel.VIRTUAL);
                  baseUI.getPrimaryScene().addRenderableProvider(rootJointGraphic, RDXSceneLevel.VIRTUAL);

                  baseUI.getPerspectiveManager().reloadPerspective();
                  navigationPanel.setRenderMethod(this::renderNavigationPanel);
               }

               if (initialized)
               {
                  updateRapidRegionsExtractor();
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

            if (imgui.internal.ImGui.button("Load Previous"))
            {
               frameIndex.set(Math.max(0, frameIndex.get() - 1));
               changed = true;
            }
            imgui.internal.ImGui.sameLine();
            if (imgui.internal.ImGui.button("Load Next"))
            {
               frameIndex.set(frameIndex.get() + 1);
               changed = true;
            }

            if (changed || !initialized)
            {
               if((frameIndex.get() % HDF5Manager.MAX_BUFFER_SIZE) != (HDF5Manager.MAX_BUFFER_SIZE - 1))
               {
                  perceptionDataLoader.loadCompressedDepth(sensorTopicName, frameIndex.get(), bytedecoDepthImage.getBytedecoOpenCVMat());
                  updateRapidRegionsExtractor();
               }

               updatePointCloudRenderer();
               rapidPlanarRegionsExtractor.getDebugger().getDebugPoints().clear();

               if (!initialized)
               {
                  initialized = true;
               }
            }
         }

         @Override
         public void dispose()
         {
            rapidPlanarRegionsExtractor.setProcessing(false);
            perceptionDataLoader.destroy();
            rapidRegionsUIPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   public void updatePointCloudRenderer()
   {
      if (rapidRegionsUIPanel.getPointCloudRenderEnabled())
      {
         rapidPlanarRegionsExtractor.getDebugger().transformPoints(cameraFrame.getTransformToWorldFrame());
         pointCloudRenderer.setPointsToRender(rapidPlanarRegionsExtractor.getDebugger().getDebugPoints(), Color.GRAY);
         pointCloudRenderer.updateMesh();
      }
   }

   private void updateRapidRegionsExtractor()
   {
      if (!rapidPlanarRegionsExtractor.isProcessing())
      {
         rapidPlanarRegionsExtractor.getDebugger().setShowPointCloud(rapidRegionsUIPanel.getPointCloudRenderEnabled());
         if((frameIndex.get() % HDF5Manager.MAX_BUFFER_SIZE) != (HDF5Manager.MAX_BUFFER_SIZE - 1))
         {
            ThreadTools.startAsDaemon(() ->
                                      {
                                         Point3D position = sensorPositionBuffer.get(frameIndex.get());
                                         Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());

                                         //                                      sensorTransformToWorld.set(orientation, position);
                                         cameraPose.set(position, orientation);
                                         cameraFrame.setPoseAndUpdate(cameraPose);

                                         //                                      cameraFrame.update();

                                         //LogTools.info("Transform to World: {}", cameraFrame.getTransformToWorldFrame());

                                         regionsWithPose.getPlanarRegionsList().clear();
                                         rapidPlanarRegionsExtractor.update(bytedecoDepthImage, cameraFrame, regionsWithPose);
                                      }, getClass().getSimpleName() + "RapidRegions");

         }
      }

      generateMesh(regionsWithPose.getPlanarRegionsList());

   }

   public synchronized void generateMesh(PlanarRegionsList regionsList)
   {
      if (rapidPlanarRegionsExtractor.isModified())
      {
         rapidRegionsUIPanel.render3DGraphics(regionsList, cameraFrame);
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
