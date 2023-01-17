package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.logging.HDF5Manager;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.tools.thread.Activator;

import java.util.ArrayList;

public class RDXRapidRegionsExtractionDemo implements RenderableProvider
{
   String PERCEPTION_LOG_DIRECTORY = System.getProperty("user.home") + "/.ihmc/logs/perception/";
   String PERCEPTION_LOG_FILE = "20230114_155447_PerceptionLog.hdf5";

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final RDXRapidRegionsUIPanel rapidRegionsUIPanel = new RDXRapidRegionsUIPanel();
   private ImGuiPanel navigationPanel;

   private String sensorTopicName;

   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final OpenCLFloatParameters parametersOpenCLFloatBuffer = new OpenCLFloatParameters();
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   private final RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
   private final PlanarRegionsListWithPose regionsWithPose = new PlanarRegionsListWithPose();
   ;
   private final ReferenceFrame cameraFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("l515ReferenceFrame",
                                                                                                              ReferenceFrame.getWorldFrame(),
                                                                                                              sensorTransformToWorld);
   private final ArrayList<Point3D> cameraPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> cameraOrientationBuffer = new ArrayList<>();
   ;
   private Activator nativesLoadedActivator;
   private BytedecoImage bytedecoDepthImage;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private PerceptionDataLoader perceptionDataLoader;

   private ImInt frameIndex = new ImInt(0);

   private int totalNumberOfPoints;

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
         }

         private void createOuster(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
            perceptionDataLoader.openLogFile(PERCEPTION_LOG_DIRECTORY + PERCEPTION_LOG_FILE);
            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, frameIndex.get(), bytedecoDepthImage.getBytedecoOpenCVMat());
            pointCloudRenderer.create(depthHeight * depthWidth);
            rapidPlanarRegionsExtractor.create(openCLManager, openCLProgram, depthWidth, depthHeight);

            rapidRegionsUIPanel.create(rapidPlanarRegionsExtractor);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());
         }

         private void createL515(int depthHeight, int depthWidth)
         {
            sensorTopicName = PerceptionLoggerConstants.L515_DEPTH_NAME;
            perceptionDataLoader.openLogFile(PERCEPTION_LOG_DIRECTORY + PERCEPTION_LOG_FILE);
            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.L515_DEPTH_NAME, frameIndex.get(), bytedecoDepthImage.getBytedecoOpenCVMat());
            perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, cameraPositionBuffer);
            perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, cameraOrientationBuffer);
            rapidPlanarRegionsExtractor.create(openCLManager, openCLProgram, depthWidth, depthHeight, 730.7891, 731.0859, 528.6094, 408.1602);

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

                  baseUI.getPerspectiveManager().reloadPerspective();
                  navigationPanel.setRenderMethod(this::renderNavigationPanel);

                  updatePointCloudRenderer();
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
               perceptionDataLoader.loadCompressedDepth(sensorTopicName, frameIndex.get(), bytedecoDepthImage.getBytedecoOpenCVMat());
               rapidPlanarRegionsExtractor.getDebugger().getDebugPoints().clear();

               if (!initialized)
                  initialized = true;
            }
         }

         @Override
         public void dispose()
         {
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

   /* A one-time method to convert depth map to renderable pointcloud. */
   public void submitToPointCloudRenderer(int width, int height, BytedecoImage bytedecoImage, OpenCLManager openCLManager)
   {

      openCLProgram = openCLManager.loadProgram("OusterPointCloudVisualizer");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "imageToPointCloud");

      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

      totalNumberOfPoints = height * width;
      pointCloudRenderer.create(totalNumberOfPoints);

      pointCloudVertexBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * RDXPointCloudRenderer.FLOATS_PER_VERTEX, pointCloudRenderer.getVertexBuffer());
      pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
      LogTools.info("Allocated new buffers. {} points.", totalNumberOfPoints);

      // TODO: Create tuners for these
      double verticalFieldOfView = Math.PI / 2.0;
      double horizontalFieldOfView = 2.0 * Math.PI;

      parametersOpenCLFloatBuffer.setParameter((float) horizontalFieldOfView);
      parametersOpenCLFloatBuffer.setParameter((float) verticalFieldOfView);
      parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getX32());
      parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getY32());
      parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getZ32());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM00());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM01());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM02());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM10());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM11());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM12());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM20());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM21());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM22());
      parametersOpenCLFloatBuffer.setParameter(width);
      parametersOpenCLFloatBuffer.setParameter(height);
      parametersOpenCLFloatBuffer.setParameter(0.01f);

      parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
      bytedecoImage.writeOpenCLImage(openCLManager);
      pointCloudRenderer.updateMeshFastestBeforeKernel();
      pointCloudVertexBuffer.syncWithBackingBuffer();

      openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 1, bytedecoImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(unpackPointCloudKernel, width, height);

      pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);
      pointCloudRenderer.updateMeshFastestAfterKernel();
   }

   private void updateRapidRegionsExtractor()
   {
      if (!rapidPlanarRegionsExtractor.isProcessing())
      {
         ThreadTools.startAsDaemon(() ->
                                   {

                                      Point3D position = cameraPositionBuffer.get(frameIndex.get());
                                      Quaternion orientation = cameraOrientationBuffer.get(frameIndex.get());

                                      sensorTransformToWorld.getTranslation().set(position);
                                      sensorTransformToWorld.getRotation().set(orientation);

                                      cameraFrame.update();

                                      //LogTools.info("Transform to World: {}", cameraFrame.getTransformToWorldFrame());

                                      regionsWithPose.getPlanarRegionsList().clear();
                                      rapidPlanarRegionsExtractor.update(bytedecoDepthImage, cameraFrame, regionsWithPose);
                                   }, getClass().getSimpleName() + "RapidRegions");
      }

      if (rapidPlanarRegionsExtractor.isModified())
      {
         rapidRegionsUIPanel.render3DGraphics(regionsWithPose.getPlanarRegionsList(), cameraFrame);
         rapidRegionsUIPanel.render();
         rapidPlanarRegionsExtractor.setModified(false);
         rapidPlanarRegionsExtractor.setProcessing(false);
      }
   }

   // TODO: Complete this for visualizing the patch centroids and normals
   private RDXModelInstance constructSurfelMesh()
   {
      RecyclingArrayList<Point3D32> debugPoints = rapidPlanarRegionsExtractor.getDebugger().getDebugPoints();

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();

      ArrayList<Point3DReadOnly> points = new ArrayList<>();
      for (int i = 0; i < debugPoints.size(); i++)
      {
         points.clear();

         points.add(debugPoints.get(i));

         Point3D32 point = debugPoints.get(i);
         meshBuilder.addPolygon(points, new Color(0.5f, 0.6f, 0.0f, 1.0f)); // dark red
      }

      Mesh mesh = meshBuilder.generateMesh();
      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL20.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(com.badlogic.gdx.graphics.Color.WHITE));
      modelBuilder.part(meshPart, material);

      Model model = modelBuilder.end();

      RDXModelInstance modelInstance = new RDXModelInstance(model);
      return modelInstance;
   }

   public static void main(String[] args)
   {
      new RDXRapidRegionsExtractionDemo();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (rapidRegionsUIPanel.getPointCloudRenderEnabled())
         pointCloudRenderer.getRenderables(renderables, pool);

      if (rapidRegionsUIPanel.get3DPlanarRegionsRenderEnabled())
         rapidRegionsUIPanel.getRenderables(renderables, pool);
   }
}
