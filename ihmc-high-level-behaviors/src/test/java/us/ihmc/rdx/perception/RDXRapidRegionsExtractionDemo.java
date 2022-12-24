package us.ihmc.rdx.perception;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.avatar.gpuPlanarRegions.RapidPlanarRegionsExtractor;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.thread.Activator;

public class RDXRapidRegionsExtractionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final OpenCLFloatParameters parametersOpenCLFloatBuffer = new OpenCLFloatParameters();
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   private final GPUPlanarRegionExtractionParameters gpuPlanarRegionExtractionParameters = new GPUPlanarRegionExtractionParameters();
   private final RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(gpuPlanarRegionExtractionParameters);
   private Activator nativesLoadedActivator;
   private BytedecoImage bytedecoDepthImage;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private PerceptionDataLoader perceptionDataLoader;

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


            //submitToPointCloudRenderer(depthWidth, depthHeight, bytedecoDepthImage, openCLManager);
            submitToPlanarRegionExtractor(depthWidth, depthHeight, bytedecoDepthImage, openCLManager);
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
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

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

   public void submitToPlanarRegionExtractor(int width, int height, BytedecoImage bytedecoImage, OpenCLManager openCLManager)
   {
      rapidPlanarRegionsExtractor.create(openCLManager, bytedecoImage, width, height);
      planarRegionsGraphic = new RDXPlanarRegionsGraphic();

      // Get the planar regions from the planar region extractor
      rapidPlanarRegionsExtractor.update();
      PlanarRegionsList planarRegions = rapidPlanarRegionsExtractor.getPlanarRegionsList();

      // Submit the planar regions to the planar region renderer
      planarRegionsGraphic.generateMeshes(planarRegions);
      planarRegionsGraphic.update();

   }

   public static void main(String[] args)
   {
      new RDXRapidRegionsExtractionDemo();
   }
}
