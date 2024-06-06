package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_tracking.TrackerCSRT;
import org.bytedeco.opencv.opencv_tracking.TrackerKCF;
import org.bytedeco.opencv.opencv_video.Tracker;
import org.bytedeco.opencv.opencv_video.TrackerNano;
import org.bytedeco.opencv.opencv_video.TrackerNano.Params;
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
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.concurrent.atomic.AtomicBoolean;

public class RDXOpenCVTrackerDemo
{
   private final Params nanoParams = new TrackerNano.Params();

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;

   private final ZEDColorDepthImageRetriever imageRetriever;
   private final ZEDColorDepthImagePublisher imagePublisher;

   private final RDXBaseUI baseUI = new RDXBaseUI("OpenCV Tracker Demo");
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();

   private final ImInt currentTracker = new ImInt(0);
   private final String[] availableTrackers = {"CSRT", "KCF", "Nano"};
   private final Notification changeTracker = new Notification();

   private Tracker tracker;
   private Rect regionOfInterest = null;
   private Rect lastDetection = null;
   private final Notification roiSelectionRequest = new Notification();

   private AtomicBoolean selectinRegionOfInterest = new AtomicBoolean(false);
   private boolean done = false;

   public RDXOpenCVTrackerDemo()
   {
      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "kcf_tracker_demo");
      ros2Helper = new ROS2Helper(ros2Node);

      imageRetriever = new ZEDColorDepthImageRetriever(0,
                                                       ReferenceFrame::getWorldFrame,
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      imagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH, PerceptionAPI.ZED2_CUT_OUT_DEPTH);

      tracker = TrackerKCF.create();

      WorkspaceResourceDirectory directory = new WorkspaceResourceDirectory(RDXOpenCVTrackerDemo.class, "/openCVTrackers");
      WorkspaceFile nanoBackboneFile = new WorkspaceFile(directory, "nano_tracker_backbone.onnx");
      WorkspaceFile nanoNeckheadFile = new WorkspaceFile(directory, "nano_tracker_neckhead.onnx");

      nanoParams.backbone(new BytePointer(nanoBackboneFile.getFilesystemFile().toString()));
      nanoParams.neckhead(new BytePointer(nanoNeckheadFile.getFilesystemFile().toString()));

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      while (!done)
         runStuff();

      destroy();
   }

   private void runStuff()
   {
      RawImage colorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);

      if (changeTracker.poll())
      {
         tracker.deallocate();
         switch (currentTracker.get())
         {
            case 0 -> tracker = TrackerCSRT.create();
            case 1 -> tracker = TrackerKCF.create();
            case 2 -> tracker = TrackerNano.create(nanoParams);
         }
         if (regionOfInterest != null && !regionOfInterest.empty() && !(regionOfInterest.width() == 0 || regionOfInterest.height() == 0))
            tracker.init(colorImage.getCpuImageMat(), regionOfInterest);
      }

      if (roiSelectionRequest.poll())
      {
         regionOfInterest = opencv_highgui.selectROI(colorImage.getCpuImageMat());
         opencv_highgui.destroyAllWindows();
         selectinRegionOfInterest.set(false);

         if (!regionOfInterest.empty() && !(regionOfInterest.width() == 0 || regionOfInterest.height() == 0))
         {
            tracker.deallocate();
            switch (currentTracker.get())
            {
               case 0 -> tracker = TrackerCSRT.create();
               case 1 -> tracker = TrackerKCF.create();
               case 2 -> tracker = TrackerNano.create(nanoParams);
            }
            tracker.init(colorImage.getCpuImageMat(), regionOfInterest);
         }
      }

      if (regionOfInterest != null && !regionOfInterest.empty() && !(regionOfInterest.width() == 0 || regionOfInterest.height() == 0))
      {
         lastDetection = regionOfInterest;
         if (tracker.update(colorImage.getCpuImageMat(), regionOfInterest))
         {
            opencv_imgproc.rectangle(colorImage.getCpuImageMat(), regionOfInterest, new Scalar(0.0, 255.0, 0.0, 255.0));
         }
         else
         {
            opencv_imgproc.rectangle(colorImage.getCpuImageMat(), lastDetection, new Scalar(0.0, 0.0, 255.0, 255.0));
         }

         if (tracker instanceof TrackerNano nanoTracker)
         {
            opencv_imgproc.putText(colorImage.getCpuImageMat(),
                                   new BytePointer("Confidence: " + nanoTracker.getTrackingScore()),
                                   regionOfInterest.tl(),
                                   opencv_imgproc.FONT_HERSHEY_SIMPLEX,
                                   1.0,
                                   new Scalar(0.0, 255.0, 0.0, 255.0),
                                   2,
                                   opencv_imgproc.LINE_8,
                                   false);
         }
         colorImage.getGpuImageMat().upload(colorImage.getCpuImageMat());
      }

      imagePublisher.setNextColorImage(colorImage.get(), RobotSide.LEFT);

      colorImage.release();
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
            RDXROS2ImageMessageVisualizer colorImageVisualizer = new RDXROS2ImageMessageVisualizer("Color Image",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            colorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_COLOR);
            perceptionVisualizerPanel.addVisualizer(colorImageVisualizer);

            perceptionVisualizerPanel.create();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSettings);
            baseUI.create();
         }

         @Override
         public void render()
         {
            if (!selectinRegionOfInterest.get())
            {
               perceptionVisualizerPanel.update();
               baseUI.renderBeforeOnScreenUI();
               baseUI.renderEnd();
            }
         }

         private void renderSettings()
         {
            if (ImGui.button("Select ROI"))
            {
               roiSelectionRequest.set();
               selectinRegionOfInterest.set(true);
            }

            if (ImGui.combo("Tracker Type", currentTracker, availableTrackers))
            {
               changeTracker.set();
            }
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
      new RDXOpenCVTrackerDemo();
   }
}
