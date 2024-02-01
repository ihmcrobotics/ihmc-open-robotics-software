package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.ouster.OusterNetServer;
import us.ihmc.perception.ouster.OusterDepthExtractionKernel;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.rdx.ui.graphics.RDXColorGradientMode;
import us.ihmc.rdx.ui.graphics.RDXOusterFisheyeColoredPointCloudKernel;
import us.ihmc.rdx.ui.interactable.RDXInteractableOuster;
import us.ihmc.rdx.imgui.ImPlotFrequencyPlot;
import us.ihmc.rdx.imgui.ImPlotStopwatchPlot;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

import java.nio.ByteOrder;
import java.util.Set;

public class RDXNettyOusterUI
{
   private OusterNetServer ouster;
   private RDXBytedecoImagePanel imagePanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private OpenCLManager openCLManager;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OusterDepthExtractionKernel depthExtractionKernel;
   private RDXOusterFisheyeColoredPointCloudKernel ousterFisheyeKernel;
   private RDXPointCloudRenderer pointCloudRenderer;
   private final ImFloat pointSize = new ImFloat(0.01f);
   private MutableReferenceFrame sensorFrame;
   private RDXInteractableOuster ousterInteractable;
   private int depthWidth;
   private int depthHeight;
   private BytedecoImage fisheyeImage;
   private double fisheyeFocalLengthPixelsX;
   private double fisheyeFocalLengthPixelsY;
   private double fisheyePrincipalPointPixelsX;
   private double fisheyePrincipalPointPixelsY;
   private volatile boolean isReady = false;
   private final ImPlotFrequencyPlot frameReadFrequency = new ImPlotFrequencyPlot("Frame read frequency");
   private final ImPlotStopwatchPlot depthExtractionSynchronizedBlockStopwatchPlot = new ImPlotStopwatchPlot("Depth extraction kernel block");
   private final ImPlotStopwatchPlot depthExtractionKernelStopwatchPlot = new ImPlotStopwatchPlot("Depth extraction kernel");
   private final ImPlotStopwatchPlot drawDepthImageStopwatchPlot = new ImPlotStopwatchPlot("Draw depth image");
   private final ImPlotStopwatchPlot depthImageToPointCloudStopwatchPlot = new ImPlotStopwatchPlot("Image to point cloud kernel");
   private final Notification newFrameAvailable = new Notification();
   private final ImInt levelOfColorDetail = new ImInt(0);
   private final Notification levelOfColorDetailChanged = new Notification();

   public void create(RDXBaseUI baseUI)
   {
      sensorFrame = new MutableReferenceFrame(ReferenceFrame.getWorldFrame());
      ousterInteractable = new RDXInteractableOuster(baseUI.getPrimary3DPanel(),
                                                     sensorFrame.getReferenceFrame(),
                                                     sensorFrame.getTransformToParent());
      ouster = new OusterNetServer();
      ouster.setOnFrameReceived(this::onFrameReceived);
      ouster.start();

      openCLManager = new OpenCLManager();
      ousterFisheyeKernel = new RDXOusterFisheyeColoredPointCloudKernel(openCLManager);
   }

   public boolean isOusterInitialized()
   {
      return openCLManager != null && ouster.isInitialized();
   }

   public void createAfterOusterInitialized()
   {
      depthWidth = ouster.getImageWidth();
      depthHeight = ouster.getImageHeight();
      imagePanel = new RDXBytedecoImagePanel("Ouster Depth Image", depthWidth, depthHeight);
      createPointCloudAndKernel();
      isReady = true;
   }

   /**
    * This method can be called multiple times from the UI thread when the parameters require buffer sizes to change.
    * It will automatically calculate the new sizes of everything.
    */
   private void createPointCloudAndKernel()
   {
      depthExtractionKernel = new OusterDepthExtractionKernel(ouster, openCLManager, () -> false, () -> false);

      int totalNumberOfPoints = ousterFisheyeKernel.calculateNumberOfPointsForLevelOfColorDetail(ouster.getImageWidth(),
                                                                                                 ouster.getImageHeight(),
                                                                                                 levelOfColorDetail.get());

      if (pointCloudVertexBuffer == null
          || pointCloudVertexBuffer.getBackingDirectFloatBuffer().capacity() / RDXPointCloudRenderer.FLOATS_PER_VERTEX != totalNumberOfPoints)
      {
         LogTools.info("Allocating new buffers. {} total points", totalNumberOfPoints);

         if (pointCloudRenderer != null)
            pointCloudRenderer.dispose();
         pointCloudRenderer = new RDXPointCloudRenderer();
         pointCloudRenderer.create(totalNumberOfPoints);

         pointCloudVertexBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * RDXPointCloudRenderer.FLOATS_PER_VERTEX,
                                                        pointCloudRenderer.getVertexBuffer());
         pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
      }

      ousterFisheyeKernel.setInstrinsicParameters(ouster.getBeamAltitudeAnglesBuffer(), ouster.getBeamAzimuthAnglesBuffer());
   }

   public void setFisheyeImageToColorPoints(BytedecoImage fThetaFisheyeRGBA8Image,
                                            double fisheyeFocalLengthPixelsX,
                                            double fisheyeFocalLengthPixelsY,
                                            double fisheyePrincipalPointPixelsX,
                                            double fisheyePrincipalPointPixelsY)
   {
      this.fisheyeFocalLengthPixelsX = fisheyeFocalLengthPixelsX;
      this.fisheyeFocalLengthPixelsY = fisheyeFocalLengthPixelsY;
      this.fisheyePrincipalPointPixelsX = fisheyePrincipalPointPixelsX;
      this.fisheyePrincipalPointPixelsY = fisheyePrincipalPointPixelsY;

      if (fisheyeImage == null)
      {
         fisheyeImage = new BytedecoImage(fThetaFisheyeRGBA8Image.getImageWidth(), fThetaFisheyeRGBA8Image.getImageHeight(), opencv_core.CV_8UC4);
         fisheyeImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      }
      else
      {
         fisheyeImage.ensureDimensionsMatch(fThetaFisheyeRGBA8Image, openCLManager);
      }

      fThetaFisheyeRGBA8Image.getBytedecoOpenCVMat().copyTo(fisheyeImage.getBytedecoOpenCVMat());
   }

   public void update()
   {
      if (newFrameAvailable.poll())
      {
         if (levelOfColorDetailChanged.poll())
            createPointCloudAndKernel();

         // Synchronize with copying the Ouster's buffer to the buffer used for this kernel
         // All this is included in the block because it's not clear where the actual memory
         // operations occur. Probably in the finish method.
         depthExtractionSynchronizedBlockStopwatchPlot.start();
         synchronized (this)
         {
            depthExtractionSynchronizedBlockStopwatchPlot.stop();
            depthExtractionKernelStopwatchPlot.start();
            depthExtractionKernel.runKernel(ousterInteractable.getInteractableFrameModel().getReferenceFrame().getTransformToRoot());
            depthExtractionKernelStopwatchPlot.stop();
         }

         drawDepthImageStopwatchPlot.start();
         imagePanel.drawDepthImage(depthExtractionKernel.getExtractedDepthImage().getBytedecoOpenCVMat());
         drawDepthImageStopwatchPlot.stop();

         pointCloudRenderer.updateMeshFastestBeforeKernel();
         pointCloudVertexBuffer.syncWithBackingBuffer(); // TODO: Is this necessary?

         ousterFisheyeKernel.updateSensorTransform(ousterInteractable.getInteractableFrameModel().getReferenceFrame());
         depthImageToPointCloudStopwatchPlot.start();
         ousterFisheyeKernel.runKernel(ouster.getLidarOriginToBeamOrigin(),
                                       pointSize.get(),
                                       true,
                                       RDXColorGradientMode.WORLD_Z.ordinal(),
                                       true,
                                       depthExtractionKernel.getExtractedDepthImage(),
                                       fisheyeFocalLengthPixelsX,
                                       fisheyeFocalLengthPixelsY,
                                       fisheyePrincipalPointPixelsX,
                                       fisheyePrincipalPointPixelsY,
                                       fisheyeImage,
                                       pointCloudVertexBuffer);
         depthImageToPointCloudStopwatchPlot.stop();

         pointCloudRenderer.updateMeshFastestAfterKernel();
      }
   }

   private synchronized void onFrameReceived()
   {
      if (isReady)
      {
         depthExtractionKernel.copyLidarFrameBuffer();
         frameReadFrequency.ping();
         newFrameAvailable.set();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

      ImGui.sliderFloat(labels.get("Point size"), pointSize.getData(), 0.0005f, 0.05f);
      if (ImGui.sliderInt(labels.get("Level of color detail"), levelOfColorDetail.getData(), 0, 3))
         levelOfColorDetailChanged.set();

      if (isOusterInitialized())
      {
         ImGui.text("Columns per frame: " + ouster.getColumnsPerFrame());
         ImGui.text("Pixels per column: " + ouster.getPixelsPerColumn());
         frameReadFrequency.renderImGuiWidgets();
         depthExtractionSynchronizedBlockStopwatchPlot.renderImGuiWidgets();
         depthExtractionKernelStopwatchPlot.renderImGuiWidgets();
         drawDepthImageStopwatchPlot.renderImGuiWidgets();
         depthImageToPointCloudStopwatchPlot.renderImGuiWidgets();
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.MODEL))
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      openCLManager.destroy();
      ouster.destroy();
   }

   public RDXBytedecoImagePanel getImagePanel()
   {
      return imagePanel;
   }

   public MutableReferenceFrame getSensorFrame()
   {
      return sensorFrame;
   }

   public RDXInteractableFrameModel getOusterInteractable()
   {
      return ousterInteractable.getInteractableFrameModel();
   }

   public RDXOusterFisheyeColoredPointCloudKernel getOusterFisheyeKernel()
   {
      return ousterFisheyeKernel;
   }

   public boolean getIsReady()
   {
      return isReady;
   }
}
