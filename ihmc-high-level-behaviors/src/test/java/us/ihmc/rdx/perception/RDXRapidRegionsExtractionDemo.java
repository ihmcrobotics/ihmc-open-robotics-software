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
import us.ihmc.avatar.gpuPlanarRegions.RapidPlanarRegionsCustomizer;
import us.ihmc.avatar.gpuPlanarRegions.RapidPlanarRegionsExtractor;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.tools.thread.Activator;

import java.util.ArrayList;

public class RDXRapidRegionsExtractionDemo implements RenderableProvider
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final RDXRapidRegionsUIPanel rapidRegionsUIPanel = new RDXRapidRegionsUIPanel();
   private ImGuiPanel navigationPanel;

   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final OpenCLFloatParameters parametersOpenCLFloatBuffer = new OpenCLFloatParameters();
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   private final RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
   private final RapidPlanarRegionsCustomizer rapidPlanarRegionsCustomizer = new RapidPlanarRegionsCustomizer();

   private Activator nativesLoadedActivator;
   private BytedecoImage bytedecoDepthImage;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private PerceptionDataLoader perceptionDataLoader;

   private ImInt frameIndex = new ImInt(300);

   private int depthWidth;
   private int depthHeight;
   private int totalNumberOfPoints;
   private RDXPlanarRegionsGraphic planarRegionsGraphic;

   public RDXRapidRegionsExtractionDemo()
   {
      depthHeight = 128;
      depthWidth = 2048;

      perceptionDataLoader = new PerceptionDataLoader();
      perceptionDataLoader.openLogFile(System.getProperty("user.home") + "/.ihmc/logs/perception/20230102_152006_PerceptionLog.hdf5");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
            baseUI.create();

            planarRegionsGraphic = new RDXPlanarRegionsGraphic();

            openCLManager = new OpenCLManager();
            openCLManager.create();

            bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, frameIndex.get(), bytedecoDepthImage.getBytedecoOpenCVMat());

            pointCloudRenderer.create(depthHeight * depthWidth);

            rapidPlanarRegionsExtractor.create(openCLManager, bytedecoDepthImage, depthWidth, depthHeight);
            planarRegionsGraphic = new RDXPlanarRegionsGraphic();

            rapidRegionsUIPanel.create(rapidPlanarRegionsExtractor, rapidPlanarRegionsCustomizer);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());

            navigationPanel = new ImGuiPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            //submitToPointCloudRenderer(depthWidth, depthHeight, bytedecoDepthImage, openCLManager);
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

                  updateRapidRegionsExtractor(true);
                  updatePointCloudRenderer(true);
               }

               updateRapidRegionsExtractor(false);
               updatePointCloudRenderer(false);

               rapidRegionsUIPanel.render();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderNavigationPanel()
         {
            boolean changed = ImGui.sliderInt("Frame Index",
                                              frameIndex.getData(),
                                              0,
                                              (int) (perceptionDataLoader.getHDF5Manager().getCount(PerceptionLoggerConstants.OUSTER_DEPTH_NAME) - 1));

            if(imgui.internal.ImGui.button("Load Previous"))
            {
               frameIndex.set(Math.max(0, frameIndex.get() - 1));
               changed = true;
            }
            imgui.internal.ImGui.sameLine();
            if(imgui.internal.ImGui.button("Load Next"))
            {
               frameIndex.set(frameIndex.get() + 1);
               changed = true;
            }

            if(changed)
            {
               LogTools.info("Changed frame index to {}", frameIndex.get());
               perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, frameIndex.get(), bytedecoDepthImage.getBytedecoOpenCVMat());
               rapidPlanarRegionsExtractor.getDebugger().getDebugPoints().clear();
               updateRapidRegionsExtractor(true);
               updatePointCloudRenderer(true);
            }
         }

         @Override
         public void dispose()
         {
            perceptionDataLoader.destroy();
            baseUI.dispose();
         }
      });
   }

   public void updatePointCloudRenderer(boolean forceUpdate)
   {
      if(rapidRegionsUIPanel.getPointCloudRenderEnabled())
      {
         pointCloudRenderer.setPointsToRender(rapidPlanarRegionsExtractor.getDebugger().getDebugPoints(), Color.GRAY);
         pointCloudRenderer.updateMesh();
      }
   }

   /* A one-time method to convert depth map to renderable pointcloud. */
   public void submitToPointCloudRenderer(int width, int height, BytedecoImage bytedecoImage, OpenCLManager openCLManager)
   {
      depthWidth = width;
      depthHeight = height;

      openCLProgram = openCLManager.loadProgram("OusterPointCloudVisualizer");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "imageToPointCloud");

      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

      totalNumberOfPoints = depthWidth * depthHeight;
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
      parametersOpenCLFloatBuffer.setParameter(depthWidth);
      parametersOpenCLFloatBuffer.setParameter(depthHeight);
      parametersOpenCLFloatBuffer.setParameter(0.01f);

      parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
      bytedecoImage.writeOpenCLImage(openCLManager);
      pointCloudRenderer.updateMeshFastestBeforeKernel();
      pointCloudVertexBuffer.syncWithBackingBuffer();

      openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 1, bytedecoImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(unpackPointCloudKernel, depthWidth, depthHeight);

      pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);
      pointCloudRenderer.updateMeshFastestAfterKernel();
   }

   private void updateRapidRegionsExtractor(boolean changed)
   {

      // Get the planar regions from the planar region extractor
      PlanarRegionsListWithPose regionsWithPose = new PlanarRegionsListWithPose();
      rapidPlanarRegionsExtractor.update(changed);
      rapidPlanarRegionsCustomizer.createCustomPlanarRegionsList(rapidPlanarRegionsExtractor.getGPUPlanarRegions(),
                                                                 ReferenceFrame.getWorldFrame(),
                                                                 regionsWithPose);

      PlanarRegionsList planarRegions = regionsWithPose.getPlanarRegionsList();

      // Submit the planar regions to the planar region renderer
      planarRegionsGraphic.generateMeshes(planarRegions);
      planarRegionsGraphic.update();
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
      if(rapidRegionsUIPanel.getPointCloudRenderEnabled())
         pointCloudRenderer.getRenderables(renderables, pool);

      if(rapidRegionsUIPanel.get3DPlanarRegionsRenderEnabled())
         planarRegionsGraphic.getRenderables(renderables, pool);
   }
}
