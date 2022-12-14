package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import imgui.ImGui;
import imgui.type.ImFloat;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.*;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.rdx.ui.gizmo.CylinderRayIntersection;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class RDXNettyOusterUI
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private NettyOuster ouster;
   private RDXCVImagePanel imagePanel;
   private final FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private int numberOfDepthPoints;
   private ByteBuffer lidarFrameByteBufferCopy;
   private BytePointer lidarFrameByteBufferPointerCopy;
   private BytePointer lidarFrameByteBufferPointer;
   private OpenCLManager openCLManager;
   private _cl_program depthImageExtractionProgram;
   private _cl_kernel extractDepthImageKernel;
   private _cl_mem lidarFrameBufferObject;
   private BytedecoImage extractedDepthImage;
   private _cl_program pointCloudRenderingProgram;
   private _cl_kernel imageToPointCloudKernel;
   private OpenCLIntBuffer pixelShiftOpenCLBuffer;
   private final OpenCLFloatParameters depthImageKernelParameters = new OpenCLFloatParameters();
   private final OpenCLFloatParameters imageToPointCloudParameters = new OpenCLFloatParameters();
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private RDXPointCloudRenderer pointCloudRenderer;
   private final ImFloat verticalFieldOfView = new ImFloat((float) Math.toRadians(90.0));
   private final ImFloat horizontalFieldOfView = new ImFloat((float) Math.toRadians(360.0));
   private RDXInteractableFrameModel ousterInteractable;
   private int depthWidth;
   private int depthHeight;
   private volatile boolean isReady = false;

   public RDXNettyOusterUI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("Ouster", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            ModelData ousterSensorModel = RDXModelLoader.loadModelData("environmentObjects/ousterSensor/Ouster.g3dj");
            CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();
            ousterInteractable = new RDXInteractableFrameModel();
            ousterInteractable.create(ReferenceFrame.getWorldFrame(),
                                      baseUI.getPrimary3DPanel(),
                                      ousterSensorModel,
                                      pickRay ->
            {
               cylinderIntersection.update(0.0734, 0.04, -0.0372, ousterInteractable.getReferenceFrame().getTransformToWorldFrame());
               return cylinderIntersection.intersect(pickRay);
            });


            ouster = new NettyOuster();
            ouster.setOnFrameReceived(this::onFrameReceived);
            ouster.bind();
         }

         private synchronized void onFrameReceived()
         {
            if (isReady)
            {
               lidarFrameByteBufferPointer.position(0);
               lidarFrameByteBufferPointerCopy.position(0);
               lidarFrameByteBufferPointerCopy.put(lidarFrameByteBufferPointer);

               frameReadFrequency.ping();
            }
         }

         /**
          * The threading here isn't really ideal. The rendered image and point cloud
          * only need to be rendered after onFrameReceived in a new thread. But I didn't
          * find it necessary to spend the time on the thread barriers for those yet,
          * so we just run the kernels everytime and sync over the copy.
          */
         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  openCLManager = new OpenCLManager();
                  openCLManager.create();
               }

               if (openCLManager != null && ouster.isInitialized())
               {
                  if (imagePanel == null)
                  {
                     depthWidth = ouster.getImageWidth();
                     depthHeight = ouster.getImageHeight();
                     imagePanel = new RDXCVImagePanel("Ouster Depth Image", depthWidth, depthHeight);

                     baseUI.getImGuiPanelManager().addPanel(imagePanel.getVideoPanel());
                     baseUI.getPerspectiveManager().reloadPerspective();

                     numberOfDepthPoints = ouster.getImageWidth() * ouster.getImageHeight();

                     lidarFrameByteBufferCopy = ByteBuffer.allocateDirect(ouster.getLidarFrameByteBuffer().limit());
                     lidarFrameByteBufferPointerCopy = new BytePointer(lidarFrameByteBufferCopy);
                     lidarFrameByteBufferPointer = new BytePointer(ouster.getLidarFrameByteBuffer());

                     extractedDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
                     depthImageExtractionProgram = openCLManager.loadProgram("OusterDepthImagePublisher");
                     extractDepthImageKernel = openCLManager.createKernel(depthImageExtractionProgram, "extractDepthImage");
                     lidarFrameBufferObject = openCLManager.createBufferObject(lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);
                     extractedDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
                     pixelShiftOpenCLBuffer = new OpenCLIntBuffer(ouster.getPixelShiftBuffer());
                     pixelShiftOpenCLBuffer.createOpenCLBufferObject(openCLManager);

                     pointCloudRenderingProgram = openCLManager.loadProgram("OusterPointCloudVisualizer");
                     imageToPointCloudKernel = openCLManager.createKernel(pointCloudRenderingProgram, "imageToPointCloud");

                     pointCloudRenderer = new RDXPointCloudRenderer();
                     pointCloudRenderer.create(numberOfDepthPoints);
                     baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer, RDXSceneLevel.MODEL);

                     pointCloudVertexBuffer = new OpenCLFloatBuffer(numberOfDepthPoints * RDXPointCloudRenderer.FLOATS_PER_VERTEX,
                                                                    pointCloudRenderer.getVertexBuffer());
                     pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);

                     isReady = true;
                  }

                  depthImageKernelParameters.setParameter(ouster.getColumnsPerFrame());
                  depthImageKernelParameters.setParameter(ouster.getMeasurementBlockSize());
                  depthImageKernelParameters.setParameter(NettyOuster.HEADER_BLOCK_BYTES);
                  depthImageKernelParameters.setParameter(NettyOuster.CHANNEL_DATA_BLOCK_BYTES);
                  depthImageKernelParameters.setParameter(NettyOuster.MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM);
                  depthImageKernelParameters.writeOpenCLBufferObject(openCLManager);

                  synchronized (this)
                  {
                     pixelShiftOpenCLBuffer.writeOpenCLBufferObject(openCLManager);
                     openCLManager.enqueueWriteBuffer(lidarFrameBufferObject, lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);

                     openCLManager.setKernelArgument(extractDepthImageKernel, 0, depthImageKernelParameters.getOpenCLBufferObject());
                     openCLManager.setKernelArgument(extractDepthImageKernel, 1, pixelShiftOpenCLBuffer.getOpenCLBufferObject());
                     openCLManager.setKernelArgument(extractDepthImageKernel, 2, lidarFrameBufferObject);
                     openCLManager.setKernelArgument(extractDepthImageKernel, 3, extractedDepthImage.getOpenCLImageObject());
                     openCLManager.execute2D(extractDepthImageKernel, depthWidth, depthHeight);
                     extractedDepthImage.readOpenCLImage(openCLManager);
                     openCLManager.finish();
                  }

                  imagePanel.drawDepthImage(extractedDepthImage.getBytedecoOpenCVMat());

                  RigidBodyTransform transformToWorldFrame = ousterInteractable.getReferenceFrame().getTransformToWorldFrame();

                  imageToPointCloudParameters.setParameter(horizontalFieldOfView.get());
                  imageToPointCloudParameters.setParameter(verticalFieldOfView.get());
                  imageToPointCloudParameters.setParameter(transformToWorldFrame.getTranslation().getX32());
                  imageToPointCloudParameters.setParameter(transformToWorldFrame.getTranslation().getY32());
                  imageToPointCloudParameters.setParameter(transformToWorldFrame.getTranslation().getZ32());
                  imageToPointCloudParameters.setParameter((float) transformToWorldFrame.getRotation().getM00());
                  imageToPointCloudParameters.setParameter((float) transformToWorldFrame.getRotation().getM01());
                  imageToPointCloudParameters.setParameter((float) transformToWorldFrame.getRotation().getM02());
                  imageToPointCloudParameters.setParameter((float) transformToWorldFrame.getRotation().getM10());
                  imageToPointCloudParameters.setParameter((float) transformToWorldFrame.getRotation().getM11());
                  imageToPointCloudParameters.setParameter((float) transformToWorldFrame.getRotation().getM12());
                  imageToPointCloudParameters.setParameter((float) transformToWorldFrame.getRotation().getM20());
                  imageToPointCloudParameters.setParameter((float) transformToWorldFrame.getRotation().getM21());
                  imageToPointCloudParameters.setParameter((float) transformToWorldFrame.getRotation().getM22());
                  imageToPointCloudParameters.setParameter(depthWidth);
                  imageToPointCloudParameters.setParameter(depthHeight);
                  imageToPointCloudParameters.setParameter(0.01f);
                  imageToPointCloudParameters.writeOpenCLBufferObject(openCLManager);

                  extractedDepthImage.writeOpenCLImage(openCLManager);
                  pointCloudRenderer.updateMeshFastestBeforeKernel();
                  pointCloudVertexBuffer.syncWithBackingBuffer();

                  openCLManager.setKernelArgument(imageToPointCloudKernel, 0, imageToPointCloudParameters.getOpenCLBufferObject());
                  openCLManager.setKernelArgument(imageToPointCloudKernel, 1, extractedDepthImage.getOpenCLImageObject());
                  openCLManager.setKernelArgument(imageToPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
                  openCLManager.execute2D(imageToPointCloudKernel, depthWidth, depthHeight);
                  pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);

                  pointCloudRenderer.updateMeshFastestAfterKernel();
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            if (imagePanel != null)
            {
               ImGui.text("Frame read frequency: " + frameReadFrequency.getFrequency());
            }

            ImGuiTools.volatileInputFloat(labels.get("Vertical field of view"), verticalFieldOfView);
            ImGuiTools.volatileInputFloat(labels.get("Horizontal field of view"), horizontalFieldOfView);
         }

         @Override
         public void dispose()
         {
            openCLManager.destroy();
            ouster.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXNettyOusterUI();
   }
}