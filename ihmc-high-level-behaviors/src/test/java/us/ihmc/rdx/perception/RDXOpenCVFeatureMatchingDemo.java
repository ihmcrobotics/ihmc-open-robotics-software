package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_features2d;
import org.bytedeco.opencv.global.opencv_flann;
import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.ByteVectorVector;
import org.bytedeco.opencv.opencv_core.DMatchVectorVector;
import org.bytedeco.opencv.opencv_core.KeyPointVector;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_features2d.FlannBasedMatcher;
import org.bytedeco.opencv.opencv_features2d.ORB;
import org.bytedeco.opencv.opencv_flann.IndexParams;
import org.bytedeco.opencv.opencv_flann.SearchParams;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

public class RDXOpenCVFeatureMatchingDemo
{
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "feature_matching_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ZEDColorDepthImageRetriever imageRetriever;
   private final ZEDColorDepthImagePublisher imagePublisher;

   private final RDXBaseUI baseUI = new RDXBaseUI("Feature Matching Demo");
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();
   private final Notification captureFrame = new Notification();
   private final Notification selectRegionOfInterest = new Notification();
   private final AtomicBoolean selectingRegionOfInterest = new AtomicBoolean(false);
   private RawImage capturedFrame = null;
   private Mat regionOfInterestMask = null;

   private final ImInt maxFeature = new ImInt(1000);

   private final ORB orb;
   private final FlannBasedMatcher featureMatcher;

   private boolean done = false;

   public RDXOpenCVFeatureMatchingDemo()
   {
      imageRetriever = new ZEDColorDepthImageRetriever(0,
                                                       ReferenceFrame::getWorldFrame,
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      imagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH);

      orb = ORB.create();
      IndexParams indexParams = new IndexParams();
      indexParams.setAlgorithm(opencv_flann.FLANN_INDEX_LSH);
      SearchParams searchParams = new SearchParams(32, 0.1f, false);
      featureMatcher = new FlannBasedMatcher(indexParams, searchParams);

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      imageRetriever.start();
      while (!done)
         runStuff();

      destroy();
   }

   private void destroy()
   {
      imageRetriever.destroy();
      imagePublisher.destroy();
   }

   private void runStuff()
   {
      RawImage colorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);

      // Capture frame button pressed; set the first image using the current image
      if (captureFrame.poll())
      {
         capturedFrame = new RawImage(colorImage);
         // feature matching uses gray scale images
         opencv_cudaimgproc.cvtColor(capturedFrame.getGpuImageMat(), capturedFrame.getGpuImageMat(), opencv_imgproc.CV_BGR2GRAY);
         // remove the ROI, use entire frame
         if (regionOfInterestMask != null)
         {
            regionOfInterestMask.release();
            regionOfInterestMask = null;
         }
      }

      // Select ROI button pressed; set the first image using the current image & allow user to select the ROI
      if (selectRegionOfInterest.poll())
      {
         capturedFrame = new RawImage(colorImage);
         // feature matching uses gray scale images
         opencv_cudaimgproc.cvtColor(capturedFrame.getGpuImageMat(), capturedFrame.getGpuImageMat(), opencv_imgproc.CV_BGR2GRAY);

         // allow user to select ROI
         Rect regionOfInterest = opencv_highgui.selectROI(capturedFrame.getCpuImageMat());
         opencv_highgui.destroyAllWindows();
         selectingRegionOfInterest.set(false);

         if (!regionOfInterest.empty() && !(regionOfInterest.width() == 0 || regionOfInterest.height() == 0))
         {
            // create the ROI mask
            if (regionOfInterestMask != null)
               regionOfInterestMask.release();
            regionOfInterestMask = new Mat(colorImage.getImageHeight(), colorImage.getImageWidth(), opencv_core.CV_8UC1, new Scalar(0.0));
            opencv_imgproc.rectangle(regionOfInterestMask, regionOfInterest, new Scalar(255.0), opencv_imgproc.FILLED, opencv_imgproc.LINE_8, 0);
         }
      }

      if (capturedFrame != null)
      {
         try (DMatchVectorVector matches = new DMatchVectorVector();
              KeyPointVector keyPoints1 = new KeyPointVector();
              Mat descriptor1 = new Mat();
              KeyPointVector keyPoints2 = new KeyPointVector();
              Mat descriptor2 = new Mat();
              Mat grayImage = new Mat())
         {
            Mat mask = regionOfInterestMask == null ? new Mat() : regionOfInterestMask.clone();

            if (regionOfInterestMask != null)
               orb.setMaxFeatures((int) (maxFeature.get() * 0.2)); // reduce number of feature points if a mask is used (smaller ares; don't need as many points)
            // find the first feature points & descriptors
            orb.detectAndCompute(capturedFrame.getCpuImageMat(), mask, keyPoints1, descriptor1);

            // ensure we're using gray scale images
            opencv_imgproc.cvtColor(colorImage.getCpuImageMat(), grayImage, opencv_imgproc.COLOR_BGR2GRAY);
            // find the second feature points & descriptors
            orb.setMaxFeatures(maxFeature.get());
            orb.detectAndCompute(grayImage, new Mat(), keyPoints2, descriptor2);

            // compute the matches (2 sets; used for Lowe's ratio test)
            featureMatcher.knnMatch(descriptor1, descriptor2, matches, 2);

            // create a mask for only the good matches (matches that pass Lowe's ratio test)
            byte[][] matchesMask = Arrays.stream(matches.get()).parallel().map(match ->
            {
               boolean good = match.size() >= 2 && match.get(0).distance() < 0.7 * match.get(1).distance();
               return new byte[]{(byte) (good ? 1 : 0), 0};
            }).toArray(byte[][]::new);

            // draw the matches on an image
            Mat matchImageMat = new Mat(colorImage.getImageHeight(), 2 * colorImage.getImageWidth(), opencv_core.CV_8UC3);
            opencv_features2d.drawMatchesKnn(capturedFrame.getCpuImageMat(),
                                             keyPoints1,
                                             grayImage,
                                             keyPoints2,
                                             matches,
                                             matchImageMat,
                                             new Scalar(0.0, 255.0, 0.0, 255.0),
                                             new Scalar(0.0, 0.0, 255.0, 255.0),
                                             new ByteVectorVector(matchesMask),
                                             0);

            // send the image to be displayed
            RawImage matchImage = new RawImage(colorImage.getSequenceNumber(),
                                               colorImage.getAcquisitionTime(),
                                               colorImage.getImageWidth() * 2,
                                               colorImage.getImageHeight(),
                                               colorImage.getDepthDiscretization(),
                                               matchImageMat,
                                               null,
                                               matchImageMat.type(),
                                               colorImage.getFocalLengthX(),
                                               colorImage.getFocalLengthY(),
                                               colorImage.getPrincipalPointX(),
                                               colorImage.getPrincipalPointY(),
                                               colorImage.getPosition(),
                                               colorImage.getOrientation());

            imagePublisher.setNextColorImage(matchImage.get(), RobotSide.RIGHT);
            matchImage.release();
         }
      }

      imagePublisher.setNextColorImage(colorImage.get(), RobotSide.LEFT);

      colorImage.release();
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            RDXROS2ImageMessageVisualizer colorImageVisualizer = new RDXROS2ImageMessageVisualizer("Color Image",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(colorImageVisualizer, PerceptionAPI.REQUEST_ZED_COLOR);

            RDXROS2ImageMessageVisualizer featureImageVisualizer = new RDXROS2ImageMessageVisualizer("Feature Image",
                                                                                                     PubSubImplementation.FAST_RTPS,
                                                                                                     PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
            perceptionVisualizerPanel.addVisualizer(featureImageVisualizer, PerceptionAPI.REQUEST_ZED_COLOR);

            perceptionVisualizerPanel.create();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSettings);
            baseUI.create();
         }

         @Override
         public void render()
         {
            if (!selectingRegionOfInterest.get())
            {
               perceptionVisualizerPanel.update();
               baseUI.renderBeforeOnScreenUI();
               baseUI.renderEnd();
            }
         }

         private void renderSettings()
         {
            if (ImGui.button("Capture Frame"))
            {
               captureFrame.set();
            }

            if (ImGui.button("Select ROI"))
            {
               selectRegionOfInterest.set();
               selectingRegionOfInterest.set(true);
            }

            ImGui.sliderInt("Max Features", maxFeature.getData(), 10, 10000);
         }

         @Override
         public void dispose()
         {
            done = true;
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXOpenCVFeatureMatchingDemo();
   }
}
