package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
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
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.ArrayList;
import java.util.Timer;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class RDXRapidRegionsExtractionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final RDXRapidRegionsUIPanel rapidRegionsUIPanel = new RDXRapidRegionsUIPanel();

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

   private int frameIndex = 0;
   private int depthWidth;
   private int depthHeight;
   private int totalNumberOfPoints;
   private RDXPlanarRegionsGraphic planarRegionsGraphic;

   public RDXRapidRegionsExtractionDemo()
   {
      depthHeight = 128;
      depthWidth = 2048;

      perceptionDataLoader = new PerceptionDataLoader();
      perceptionDataLoader.openLogFile("/home/quantum/.ihmc/logs/perception/20221222_141507_Ouster.hdf5");

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
            perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, 0, bytedecoDepthImage.getBytedecoOpenCVMat());

            rapidPlanarRegionsExtractor.create(openCLManager, bytedecoDepthImage, depthWidth, depthHeight);
            planarRegionsGraphic = new RDXPlanarRegionsGraphic();

            rapidRegionsUIPanel.create(rapidPlanarRegionsExtractor, rapidPlanarRegionsCustomizer);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());

            submitToPointCloudRenderer(depthWidth, depthHeight, bytedecoDepthImage, openCLManager);

         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {

                  baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer, RDXSceneLevel.VIRTUAL);
                  baseUI.getPrimaryScene().addRenderableProvider(planarRegionsGraphic, RDXSceneLevel.VIRTUAL);

                  baseUI.getPerspectiveManager().reloadPerspective();

               }

               //frameIndex = (frameIndex + 1) % 8;
               frameIndex++;
               if(frameIndex % 50 == 0)
               {
                  updateRapidRegionsExtractor(0);
               }

               rapidRegionsUIPanel.render();


            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            perceptionDataLoader.destroy();
            baseUI.dispose();
         }
      });
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

   private void updateRapidRegionsExtractor(int frameIndex)
   {
      perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, frameIndex, bytedecoDepthImage.getBytedecoOpenCVMat());

      // Get the planar regions from the planar region extractor
      PlanarRegionsListWithPose regionsWithPose = new PlanarRegionsListWithPose();
      rapidPlanarRegionsExtractor.update();
      rapidPlanarRegionsCustomizer.createCustomPlanarRegionsList(rapidPlanarRegionsExtractor.getGPUPlanarRegions(),
                                                                 ReferenceFrame.getWorldFrame(),
                                                                 regionsWithPose);

      PlanarRegionsList planarRegions = regionsWithPose.getPlanarRegionsList();
      LogTools.info("Total Planar Regions in List: {}", planarRegions.getNumberOfPlanarRegions());

      //pointCloudRenderer.create(200000);
      //pointCloudRenderer.setPointsToRender(rapidPlanarRegionsExtractor.getDebugger().getDebugPoints());
      //if (!rapidPlanarRegionsExtractor.getDebugger().getDebugPoints().isEmpty())
      //{
      //   pointCloudRenderer.updateMesh();
      //}

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
}
