package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import imgui.ImGui;
import imgui.type.ImFloat;
import org.bytedeco.opencl._cl_program;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.*;
import us.ihmc.perception.ouster.OusterDepthExtractionKernel;
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
import us.ihmc.rdx.ui.graphics.RDXOusterDepthImageToPointCloudKernel;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;

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
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private OusterDepthExtractionKernel depthExtractionKernel;
   private RDXOusterDepthImageToPointCloudKernel depthImageToPointCloudKernel;
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
               depthExtractionKernel.copyLidarFrameBuffer();

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

                     openCLProgram = openCLManager.loadProgram("OusterDepthImageExtraction");
                     depthExtractionKernel = new OusterDepthExtractionKernel(ouster, openCLManager, openCLProgram);

                     pointCloudRenderer = new RDXPointCloudRenderer();
                     pointCloudRenderer.create(numberOfDepthPoints);
                     baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer, RDXSceneLevel.MODEL);

                     depthImageToPointCloudKernel = new RDXOusterDepthImageToPointCloudKernel(pointCloudRenderer,
                                                                                              openCLManager,
                                                                                              depthExtractionKernel.getExtractedDepthImage());
                     isReady = true;
                  }

                  // Synchronize with copying the Ouster's buffer to the buffer used for this kernel
                  // All this is included in the block because it's not clear where the actual memory
                  // operations occur. Probably in the finish method.
                  synchronized (this)
                  {
                     depthExtractionKernel.runKernel();
                  }

                  imagePanel.drawDepthImage(depthExtractionKernel.getExtractedDepthImage().getBytedecoOpenCVMat());

                  depthImageToPointCloudKernel.updateSensorTransform(ousterInteractable.getReferenceFrame());
                  float pointSize = 0.01f;
                  depthImageToPointCloudKernel.runKernel(horizontalFieldOfView.get(), verticalFieldOfView.get(), pointSize);
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