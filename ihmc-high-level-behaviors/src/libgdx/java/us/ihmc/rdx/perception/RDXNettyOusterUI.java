package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.perception.ouster.OusterDepthExtractionKernel;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.rdx.ui.gizmo.CylinderRayIntersection;
import us.ihmc.rdx.ui.graphics.RDXOusterDepthImageToPointCloudKernel;
import us.ihmc.rdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.ByteOrder;

public class RDXNettyOusterUI
{
   private NettyOuster ouster;
   private RDXCVImagePanel imagePanel;
   private final FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private int numberOfDepthPoints;
   private OpenCLManager openCLManager;
   private OusterDepthExtractionKernel depthExtractionKernel;
   private RDXOusterDepthImageToPointCloudKernel depthImageToPointCloudKernel;
   private RDXPointCloudRenderer pointCloudRenderer;
   private final ImFloat verticalFieldOfView = new ImFloat((float) Math.toRadians(90.0));
   private final ImFloat horizontalFieldOfView = new ImFloat((float) Math.toRadians(360.0));
   private RDXInteractableFrameModel ousterInteractable;
   private int depthWidth;
   private int depthHeight;
   private volatile boolean isReady = false;
   private final ImPlotStopwatchPlot depthExtractionSynchronizedBlockStopwatchPlot = new ImPlotStopwatchPlot("Depth extraction kernel block");
   private final ImPlotStopwatchPlot depthExtractionKernelStopwatchPlot = new ImPlotStopwatchPlot("Depth extraction kernel");
   private final ImPlotStopwatchPlot drawDepthImageStopwatchPlot = new ImPlotStopwatchPlot("Draw depth image");
   private final ImPlotStopwatchPlot depthImageToPointCloudStopwatchPlot = new ImPlotStopwatchPlot("Image to point cloud kernel");

   public void create(RDXBaseUI baseUI)
   {
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

   public void createAfterNativesLoaded()
   {
      openCLManager = new OpenCLManager();
      openCLManager.create();
   }

   public boolean isOusterInitialized()
   {
      return openCLManager != null && ouster.isInitialized();
   }

   public void createAfterOusterInitialized()
   {
      depthWidth = ouster.getImageWidth();
      depthHeight = ouster.getImageHeight();
      imagePanel = new RDXCVImagePanel("Ouster Depth Image", depthWidth, depthHeight);

      numberOfDepthPoints = ouster.getImageWidth() * ouster.getImageHeight();

      depthExtractionKernel = new OusterDepthExtractionKernel(ouster, openCLManager);

      pointCloudRenderer = new RDXPointCloudRenderer();
      pointCloudRenderer.create(numberOfDepthPoints);

      depthImageToPointCloudKernel = new RDXOusterDepthImageToPointCloudKernel(pointCloudRenderer,
                                                                               openCLManager,
                                                                               depthExtractionKernel.getExtractedDepthImage());
      isReady = true;
   }

   public void update()
   {
      // Synchronize with copying the Ouster's buffer to the buffer used for this kernel
      // All this is included in the block because it's not clear where the actual memory
      // operations occur. Probably in the finish method.
      depthExtractionSynchronizedBlockStopwatchPlot.start();
      synchronized (this)
      {
         depthExtractionSynchronizedBlockStopwatchPlot.stop();
         depthExtractionKernelStopwatchPlot.start();
         depthExtractionKernel.runKernel(ousterInteractable.getReferenceFrame().getTransformToRoot());
         depthExtractionKernelStopwatchPlot.stop();
      }

      drawDepthImageStopwatchPlot.start();
      imagePanel.drawDepthImage(depthExtractionKernel.getExtractedDepthImage().getBytedecoOpenCVMat());
      drawDepthImageStopwatchPlot.stop();

      depthImageToPointCloudKernel.updateSensorTransform(ousterInteractable.getReferenceFrame());
      depthImageToPointCloudStopwatchPlot.start();
      float pointSize = 0.01f;
      depthImageToPointCloudKernel.runKernel(horizontalFieldOfView.get(), verticalFieldOfView.get(), pointSize);
      depthImageToPointCloudStopwatchPlot.stop();
   }

   private synchronized void onFrameReceived()
   {
      if (isReady)
      {
         depthExtractionKernel.copyLidarFrameBuffer();

         frameReadFrequency.ping();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

      if (imagePanel != null)
      {
         ImGui.text("Frame read frequency: " + frameReadFrequency.getFrequency());
      }

      ImGuiTools.volatileInputFloat(labels.get("Vertical field of view"), verticalFieldOfView);
      ImGuiTools.volatileInputFloat(labels.get("Horizontal field of view"), horizontalFieldOfView);
      depthExtractionSynchronizedBlockStopwatchPlot.renderImGuiWidgets();
      depthExtractionKernelStopwatchPlot.renderImGuiWidgets();
      drawDepthImageStopwatchPlot.renderImGuiWidgets();
      depthImageToPointCloudStopwatchPlot.renderImGuiWidgets();
   }

   public void destroy()
   {
      openCLManager.destroy();
      ouster.destroy();
   }

   public RDXCVImagePanel getImagePanel()
   {
      return imagePanel;
   }

   public RDXPointCloudRenderer getPointCloudRenderer()
   {
      return pointCloudRenderer;
   }
}
