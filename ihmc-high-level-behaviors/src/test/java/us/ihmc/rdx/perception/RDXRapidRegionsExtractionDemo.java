package us.ihmc.rdx.perception;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
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
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.thread.Activator;

import java.nio.ByteBuffer;

public class RDXRapidRegionsExtractionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private Activator nativesLoadedActivator;
   private RDXEnvironmentBuilder environmentBuilder;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private GPUPlanarRegionExtractionParameters gpuPlanarRegionExtractionParameters;

   private int depthWidth;
   private int depthHeight;
   private int totalNumberOfPoints;

   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private ByteBuffer decompressionInputBuffer;

   private BytedecoImage bytedecoDepthImage;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private final OpenCLFloatParameters parametersOpenCLFloatBuffer = new OpenCLFloatParameters();
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;

   private Mat depthImage = new Mat(128, 2058, opencv_core.CV_16UC1);

   private PerceptionDataLoader perceptionDataLoader;

   public RDXRapidRegionsExtractionDemo()
   {
      perceptionDataLoader = new PerceptionDataLoader();
      perceptionDataLoader.openLogFile("/home/bmishra/.ihmc/logs/perception/20221222_141507_Ouster.hdf5");

      perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, 0, depthImage);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

            submitToPointCloudRenderer(depthImage);

            baseUI.create();

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("DemoPullDoor.json");
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  //                  rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(gpuPlanarRegionExtractionParameters);
                  //                  rapidPlanarRegionsExtractor.create(depthWidth,
                  //                                                     depthHeight, ,
                  //                                                     cameraIntrinsics.getFx(),
                  //                                                     cameraIntrinsics.getFy(),
                  //                                                     cameraIntrinsics.getCx(),
                  //                                                     cameraIntrinsics.getCy(),
                  //                                                     l515PoseGizmo.getGizmoFrame()); rapidPlanarRegionsExtractor.getEnabled().set(true);
                  //                  baseUI.getImGuiPanelManager().addPanel(rapidPlanarRegionsExtractor.getPanel());
                  //                  baseUI.getPrimaryScene().addRenderableProvider(rapidPlanarRegionsExtractor::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               pointCloudRenderer.updateMeshFastest(totalNumberOfPoints);

               //               rapidPlanarRegionsExtractor.extractPlanarRegions();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
         }
      });
   }

   public void submitToPointCloudRenderer(Mat depthImage)
   {
      depthWidth = depthImage.cols();
      depthHeight = depthImage.rows();

      totalNumberOfPoints = depthWidth * depthHeight;
      pointCloudRenderer.create(totalNumberOfPoints);

      bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      bytedecoDepthImage.resize(depthWidth, depthHeight, openCLManager, depthImage.asByteBuffer());

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
      bytedecoDepthImage.writeOpenCLImage(openCLManager);
      pointCloudVertexBuffer.syncWithBackingBuffer();

      openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 1, bytedecoDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(unpackPointCloudKernel, depthWidth, depthHeight);
      pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);
   }

   public static void main(String[] args)
   {
      new RDXRapidRegionsExtractionDemo();
   }
}
