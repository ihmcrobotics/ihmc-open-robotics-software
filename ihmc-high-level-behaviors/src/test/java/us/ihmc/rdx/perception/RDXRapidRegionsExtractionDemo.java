package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
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
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;

public class RDXRapidRegionsExtractionDemo implements RenderableProvider
{
   //20230117_161540_PerceptionLog.hdf5 (311 MB)
   //20230117_162417_PerceptionLog.hdf5 (231 MB)
   //20230117_162825_PerceptionLog.hdf5 (328 MB)

   // 'WalkForward_Rough',        # 20230228_201947_PerceptionLog.hdf5
   //'Turn_Rough',               # 20230228_202104_PerceptionLog.hdf5
   //'WalkBack_Rough',           # 20230228_202456_PerceptionLog.hdf5

   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("IROS_2023/20230228_201947_PerceptionLog.hdf5").toString();

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXRapidRegionsUIPanel rapidRegionsUIPanel = new RDXRapidRegionsUIPanel();
   private final ResettableExceptionHandlingExecutorService loadAndDecompressThreadExecutor = MissingThreadTools.newSingleThreadExecutor("LoadAndDecompress",
                                                                                                                                         true,
                                                                                                                                         1);
   private ImGuiPanel navigationPanel;

   private final RDXLineMeshModel mocapGraphic = new RDXLineMeshModel(0.02f, Color.YELLOW);
   private final RDXLineMeshModel rootJointGraphic = new RDXLineMeshModel(0.02f, Color.RED);

   private String sensorTopicName;

   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final OpenCLFloatParameters parametersOpenCLFloatBuffer = new OpenCLFloatParameters();
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   private final RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
   private final FramePlanarRegionsList frameRegions = new FramePlanarRegionsList();

   //   private final ReferenceFrame cameraFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("l515ReferenceFrame",
   //                                                                                                              ReferenceFrame.getWorldFrame(),
   //                                                                                                              sensorTransformToWorld);
   private final Pose3D cameraPose = new Pose3D();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());

   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private final ArrayList<Point3D> mocapPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> mocapOrientationBuffer = new ArrayList<>();

   private Activator nativesLoadedActivator;
   private BytedecoImage bytedecoDepthImage;
   private final BytePointer depthBytePointer = new BytePointer(1000000);

   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private PerceptionDataLoader perceptionDataLoader;

   private final ImInt frameIndex = new ImInt(0);
   private final Notification userChangedIndex = new Notification();
   private final TypedNotification<PlanarRegionsList> planarRegionsListToRenderNotification = new TypedNotification<>();

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
            openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

            navigationPanel = new ImGuiPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            createL515(720, 1280, false); // Real D455
            //createL515(768, 1024, false); // Real L515
            //            createL515(768, 1280, true); // Simulated L515

            //            createOuster(128, 1024);
         }

         private void createOuster(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);
            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     bytedecoDepthImage.getBytedecoOpenCVMat());
            //perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, sensorPositionBuffer);
            //perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, sensorOrientationBuffer);
            //perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, mocapPositionBuffer);
            //perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, mocapOrientationBuffer);

            pointCloudRenderer.create(depthHeight * depthWidth);
            rapidPlanarRegionsExtractor.create(openCLManager, openCLProgram, depthHeight, depthWidth);
            rapidPlanarRegionsExtractor.getDebugger().setEnabled(true);
            rapidPlanarRegionsExtractor.getDebugger().setShowPointCloud(false);

            rapidRegionsUIPanel.create(rapidPlanarRegionsExtractor);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());
         }

         private void createL515(int depthHeight, int depthWidth, boolean simulation)
         {
            sensorTopicName = PerceptionLoggerConstants.L515_DEPTH_NAME;
            perceptionDataLoader.openLogFile(perceptionLogFile);
            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.L515_DEPTH_NAME,
                                                     frameIndex.get(),
                                                     depthBytePointer,
                                                     bytedecoDepthImage.getBytedecoOpenCVMat());
            //perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, sensorPositionBuffer);
            //perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, sensorOrientationBuffer);
            //perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, mocapPositionBuffer);
            //perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, mocapOrientationBuffer);

            String version = simulation ? "Simulation" : "";

            // D455 Depth Intrinsics: 392.57, 392.57, 326.69, 241.13 (640 x 480 @ 60)
            // D455 Color Intrinsics: 387.42, 386.89, 321.83, 240.61 (640 x 480 @ 60)

            // D455 Depth Intrinsics: 654.29, 654.29, 651.14, 361.89 (1280 x 720 @ 30)
            // D455 Color Intrinsics: 645.69, 644.81, 643.05, 361.02 (1280 x 720 @ 30)

            // L515 Depth Intrinsics: 730.7891, 731.0859, 528.6094, 408.1602

            rapidPlanarRegionsExtractor.create(openCLManager, openCLProgram, depthHeight, depthWidth, 654.29, 654.29, 651.14, 361.89, version);
            rapidPlanarRegionsExtractor.getDebugger().setEnabled(true);

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

                  if (!mocapPositionBuffer.isEmpty())
                  {
                     MocapTools.adjustMocapPositionsByOffset(mocapPositionBuffer, sensorPositionBuffer.get(0));

                     mocapGraphic.generateMeshes(mocapPositionBuffer, 10);
                     mocapGraphic.update();

                     baseUI.getPrimaryScene().addRenderableProvider(mocapGraphic, RDXSceneLevel.VIRTUAL);
                  }

                  baseUI.getPrimaryScene().addRenderableProvider(rootJointGraphic, RDXSceneLevel.VIRTUAL);

                  rootJointGraphic.generateMeshes(sensorPositionBuffer, 5);
                  rootJointGraphic.update();

                  baseUI.getLayoutManager().reloadLayout();
                  navigationPanel.setRenderMethod(this::renderNavigationPanel);
               }

               if (userChangedIndex.poll())
               {
                  loadAndDecompressThreadExecutor.clearQueueAndExecute(() ->
                                                                       {
                                                                          //                     if ((frameIndex.get() % HDF5Manager.MAX_BUFFER_SIZE) != (HDF5Manager.MAX_BUFFER_SIZE - 1))
                                                                          {
                                                                             perceptionDataLoader.loadCompressedDepth(sensorTopicName,
                                                                                                                      frameIndex.get(),
                                                                                                                      depthBytePointer,
                                                                                                                      bytedecoDepthImage.getBytedecoOpenCVMat());
                                                                             ThreadTools.sleep(100);
                                                                          }
                                                                       });
               }

               updateRapidRegionsExtractor();
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

            if (changed)
               userChangedIndex.set();
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
         pointCloudRenderer.setPointsToRender(rapidPlanarRegionsExtractor.getDebugger().getDebugPoints(), Color.GRAY);
         pointCloudRenderer.updateMesh();
      }
   }

   private void updateRapidRegionsExtractor()
   {
      if (!rapidPlanarRegionsExtractor.isProcessing())
      {
         rapidPlanarRegionsExtractor.getDebugger().setShowPointCloud(rapidRegionsUIPanel.getPointCloudRenderEnabled());
         if ((frameIndex.get() % HDF5Manager.MAX_BUFFER_SIZE) != (HDF5Manager.MAX_BUFFER_SIZE - 1))
         {
            ThreadTools.startAsDaemon(() ->
                                      {
                                         //Point3D position = sensorPositionBuffer.get(frameIndex.get());
                                         //Quaternion orientation = sensorOrientationBuffer.get(frameIndex.get());
                                         //
                                         //// sensorTransformToWorld.set(orientation, position);
                                         //cameraPose.set(position, orientation);
                                         //cameraFrame.setPoseAndUpdate(cameraPose);

                                         // cameraFrame.update();

                                         // LogTools.info("Transform to World: {}", cameraFrame.getTransformToWorldFrame());

                                         synchronized (bytedecoDepthImage.getBytedecoOpenCVMat())
                                         {
                                            frameRegions.getPlanarRegionsList().clear();
                                            rapidPlanarRegionsExtractor.update(bytedecoDepthImage, cameraFrame, frameRegions);
                                            frameRegions.getPlanarRegionsList().applyTransform(cameraFrame.getTransformToWorldFrame());
                                            planarRegionsListToRenderNotification.set(frameRegions.getPlanarRegionsList().copy());
                                         }
                                      }, getClass().getSimpleName() + "RapidRegions");
         }
      }

      if (planarRegionsListToRenderNotification.poll())
      {
         generateMesh(planarRegionsListToRenderNotification.read());
         updatePointCloudRenderer();
      }
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
