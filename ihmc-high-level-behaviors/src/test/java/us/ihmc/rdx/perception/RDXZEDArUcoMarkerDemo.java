package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.list.array.TIntArrayList;
import imgui.type.ImFloat;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetectionResults;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetector;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.SwapReference;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class RDXZEDArUcoMarkerDemo
{
   private final ZEDColorDepthImageRetriever imageRetriever;
   private final ZEDColorDepthImagePublisher imagePublisher;
   private final MutableReferenceFrame sensorFrame;

   private final RDXBaseUI baseUI;
   private RDXMatImagePanel arUcoDetectionImagePanel;
   private final Mat annotatedImage = new Mat();
   private final RDXROS2ColoredPointCloudVisualizer pointCloudVisualizer;
   private final RDXPerceptionVisualizersPanel perceptionVisualizersPanel;
   private final RDXReferenceFrameGraphicsList markerPoseGraphics = new RDXReferenceFrameGraphicsList();
   private final ImFloat markerSize = new ImFloat(0.02975f);

   private final OpenCVArUcoMarkerDetector arUcoDetector;
   private BytedecoImage detectionImage;
   private final OpenCVArUcoMarkerDetectionResults arUcoResults;
   private final SwapReference<List<FramePose3D>> arUcoMarkerPoses = new SwapReference<>(new ArrayList<>(), new ArrayList<>());

   private volatile boolean done = false;

   public RDXZEDArUcoMarkerDemo()
   {
      imageRetriever = new ZEDColorDepthImageRetriever(0, ReferenceFrame::getWorldFrame, () -> true, () -> true, true);
      imagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH, PerceptionAPI.ZED2_CUT_OUT_DEPTH);
      sensorFrame = new MutableReferenceFrame(ReferenceFrame.getWorldFrame());

      baseUI = new RDXBaseUI("ArUco Marker Demo");
      pointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED Point Cloud",
                                                                    PubSubImplementation.FAST_RTPS,
                                                                    PerceptionAPI.ZED2_DEPTH,
                                                                    PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
      perceptionVisualizersPanel = new RDXPerceptionVisualizersPanel();

      arUcoDetector = new OpenCVArUcoMarkerDetector();
      arUcoResults = new OpenCVArUcoMarkerDetectionResults();

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      imageRetriever.start();
      while (!done)
         runDetection();

      destroy();
   }

   private void runDetection()
   {
      RawImage colorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);
      RawImage depthImage = imageRetriever.getLatestRawDepthImage();

      sensorFrame.update(transformToParent ->
      {
         transformToParent.set(imageRetriever.getLatestSensorPose());
         transformToParent.appendTranslation(0.0, imageRetriever.getZedModelData().getCenterToCameraDistance(), 0.0);
      });

      opencv_imgproc.cvtColor(colorImage.getCpuImageMat(), annotatedImage, opencv_imgproc.CV_BGR2RGB);

      if (detectionImage == null)
         initializeDetection(colorImage);

      colorImage.getCpuImageMat().copyTo(detectionImage.getBytedecoOpenCVMat());
      arUcoDetector.update();
      arUcoResults.copyOutputData(arUcoDetector);
      TIntArrayList detectedIDs = arUcoResults.getDetectedIDs();

      Set<FramePose3D> markerPoses = new HashSet<>();
      for (int i = 0; i < detectedIDs.size(); ++i)
      {
         FramePose3D markerPose = new FramePose3D();
         if (arUcoResults.getPose(detectedIDs.get(i), markerSize.get(), sensorFrame.getReferenceFrame(), ReferenceFrame.getWorldFrame(), markerPose))
            markerPoses.add(markerPose);
      }

      synchronized (arUcoMarkerPoses)
      {
         arUcoMarkerPoses.getForThreadOne().clear();
         arUcoMarkerPoses.getForThreadOne().addAll(markerPoses);
         arUcoMarkerPoses.swap();

         arUcoResults.drawDetectedMarkers(annotatedImage);
      }

      imagePublisher.setNextGpuDepthImage(depthImage.get());
      imagePublisher.setNextColorImage(colorImage.get(), RobotSide.LEFT);

      colorImage.release();
      depthImage.release();
   }

   private void initializeDetection(RawImage colorImage)
   {
      Mat image = new Mat();
      colorImage.getCpuImageMat().copyTo(image);
      detectionImage = new BytedecoImage(image);
      arUcoDetector.setSourceImageForDetection(detectionImage);
      arUcoDetector.setCameraIntrinsics(imageRetriever.getCameraIntrinsics(RobotSide.LEFT));
   }

   private void destroy()
   {
      imageRetriever.destroy();
      imagePublisher.destroy();
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            arUcoDetectionImagePanel = new RDXMatImagePanel("ArUco Detections", 1280, 720, false);
            perceptionVisualizersPanel.addVisualizer(pointCloudVisualizer);
            perceptionVisualizersPanel.create();

            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizersPanel);
            baseUI.getImGuiPanelManager().addPanel(arUcoDetectionImagePanel.getImagePanel());
            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSettings);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizersPanel);
            baseUI.getPrimaryScene().addRenderableProvider(markerPoseGraphics);
            baseUI.create();
         }

         @Override
         public void render()
         {
            perceptionVisualizersPanel.update();
            baseUI.renderBeforeOnScreenUI();

            synchronized (arUcoMarkerPoses)
            {
               List<FramePose3D> markerPoses = arUcoMarkerPoses.getForThreadTwo();

               while (markerPoseGraphics.size() > markerPoses.size())
                  markerPoseGraphics.remove(markerPoseGraphics.size() - 1);
               while (markerPoseGraphics.size() < markerPoses.size())
                  markerPoseGraphics.add(new RDXReferenceFrameGraphic(markerSize.get()));

               for (int i = 0; i < markerPoses.size(); ++i)
               {
                  markerPoseGraphics.get(i).updateFromFramePose(markerPoses.get(i));
               }

               if (!annotatedImage.isNull() && !annotatedImage.empty())
               {
                  opencv_imgproc.cvtColor(annotatedImage, arUcoDetectionImagePanel.getImage(), opencv_imgproc.CV_RGB2RGBA);
                  arUcoDetectionImagePanel.ensureDimensionsMatch(annotatedImage.cols(), annotatedImage.rows());
                  arUcoDetectionImagePanel.display();
               }
            }

            baseUI.renderEnd();
         }

         private void renderSettings()
         {
            ImGuiTools.volatileInputFloat("Marker Size", markerSize);
         }

         @Override
         public void dispose()
         {
            done = true;
            perceptionVisualizersPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   private static class RDXReferenceFrameGraphicsList extends ArrayList<RDXReferenceFrameGraphic> implements RenderableProvider
   {
      @Override
      public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
      {
         for (RDXReferenceFrameGraphic referenceFrameGraphic : this)
         {
            referenceFrameGraphic.getRenderables(renderables, pool);
         }
      }
   }

   public static void main(String[] args)
   {
      new RDXZEDArUcoMarkerDemo();
   }
}
