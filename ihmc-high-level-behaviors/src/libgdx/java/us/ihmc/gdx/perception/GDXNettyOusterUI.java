package us.ihmc.gdx.perception;

import imgui.ImGui;
import imgui.type.ImFloat;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.GDXInteractableFrameModel;
import us.ihmc.gdx.ui.gizmo.CylinderRayIntersection;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.ByteOrder;

public class GDXNettyOusterUI
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private NettyOuster ouster;
   private GDXCVImagePanel imagePanel;
   private final FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private int numberOfDepthPoints;
   private OpenCLFloatBuffer parametersBuffer;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel createPointCloudKernel;
   private OpenCLFloatBuffer pointCloudRenderingBuffer;
   private GDXPointCloudRenderer pointCloudRenderer;
   private final ImFloat verticalFieldOfView = new ImFloat((float) Math.toRadians(90.0));
   private final ImFloat horizontalFieldOfView = new ImFloat((float) Math.toRadians(360.0));
   private GDXInteractableFrameModel ousterInteractable;

   public GDXNettyOusterUI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("Ouster", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            GDXModelInstance ousterSensorModel = new GDXModelInstance(GDXModelLoader.load("environmentObjects/ousterSensor/Ouster.g3dj"));
            CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();
            ousterInteractable = new GDXInteractableFrameModel();
            ousterInteractable.create(ReferenceFrame.getWorldFrame(),
                                      baseUI.getPrimary3DPanel(),
                                      ousterSensorModel,
                                      pickRay ->
            {
               cylinderIntersection.setup(0.0734, 0.04, -0.0372, ousterInteractable.getReferenceFrame().getTransformToWorldFrame());
               return cylinderIntersection.intersect(pickRay);
            });
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  ouster = new NettyOuster();
                  ouster.bind();

                  openCLManager = new OpenCLManager();
                  openCLManager.create();
               }

               if (ouster.isInitialized())
               {
                  if (imagePanel == null)
                  {
                     imagePanel = new GDXCVImagePanel("Ouster Depth Image", ouster.getImageWidth(), ouster.getImageHeight());

                     baseUI.getImGuiPanelManager().addPanel(imagePanel.getVideoPanel());
                     baseUI.getPerspectiveManager().reloadPerspective();

                     numberOfDepthPoints = ouster.getImageWidth() * ouster.getImageHeight();

                     pointCloudRenderer = new GDXPointCloudRenderer();
                     parametersBuffer = new OpenCLFloatBuffer(16);
                     openCLProgram = openCLManager.loadProgram("OusterPointCloud");
                     createPointCloudKernel = openCLManager.createKernel(openCLProgram, "createPointCloud");

                     pointCloudRenderer.create(numberOfDepthPoints);
                     baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer, GDXSceneLevel.MODEL);

                     pointCloudRenderingBuffer = new OpenCLFloatBuffer(numberOfDepthPoints * 8, pointCloudRenderer.getVertexBuffer());
                     ouster.getDepthImageMeters().createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
                     pointCloudRenderingBuffer.createOpenCLBufferObject(openCLManager);
                     parametersBuffer.createOpenCLBufferObject(openCLManager);
                  }

                  frameReadFrequency.ping();
                  imagePanel.drawFloatImage(ouster.getDepthImageMeters().getBytedecoOpenCVMat());

                  RigidBodyTransform transformToWorldFrame = ousterInteractable.getReferenceFrame().getTransformToWorldFrame();

                  parametersBuffer.getBytedecoFloatBufferPointer().put(0, horizontalFieldOfView.get());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(1, verticalFieldOfView.get());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(2, transformToWorldFrame.getTranslation().getX32());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(3, transformToWorldFrame.getTranslation().getY32());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(4, transformToWorldFrame.getTranslation().getZ32());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(5, (float) transformToWorldFrame.getRotation().getM00());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(6, (float) transformToWorldFrame.getRotation().getM01());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(7, (float) transformToWorldFrame.getRotation().getM02());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(8, (float) transformToWorldFrame.getRotation().getM10());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(9, (float) transformToWorldFrame.getRotation().getM11());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(10, (float) transformToWorldFrame.getRotation().getM12());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(11, (float) transformToWorldFrame.getRotation().getM20());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(12, (float) transformToWorldFrame.getRotation().getM21());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(13, (float) transformToWorldFrame.getRotation().getM22());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(14, (float) ouster.getImageWidth());
                  parametersBuffer.getBytedecoFloatBufferPointer().put(15, (float) ouster.getImageHeight());

                  ouster.getDepthImageMeters().writeOpenCLImage(openCLManager);
                  parametersBuffer.writeOpenCLBufferObject(openCLManager);

                  openCLManager.setKernelArgument(createPointCloudKernel, 0, ouster.getDepthImageMeters().getOpenCLImageObject());
                  openCLManager.setKernelArgument(createPointCloudKernel, 1, pointCloudRenderingBuffer.getOpenCLBufferObject());
                  openCLManager.setKernelArgument(createPointCloudKernel, 2, parametersBuffer.getOpenCLBufferObject());
                  openCLManager.execute2D(createPointCloudKernel, ouster.getImageWidth(), ouster.getImageHeight());
                  pointCloudRenderingBuffer.readOpenCLBufferObject(openCLManager);
                  openCLManager.finish();

                  pointCloudRenderer.updateMeshFastest(numberOfDepthPoints);
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
      new GDXNettyOusterUI();
   }
}