package us.ihmc.rdx.perception;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
import imgui.ImGui;
import imgui.type.ImFloat;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Point2f;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.BallDetector;
import us.ihmc.perception.RawImage;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2TrajectoryVisualizer;
import us.ihmc.robotics.math.filters.AlphaFilteredRigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

public class RDXBallTrackingDemo
{
   private static final double BALL_DIAMETER_M = 0.0398;
   private static final double ZED_PIXEL_SIZE = 2.0E-6;

   private double zedFocalLengthM = -1.0;

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final ZEDColorDepthImageRetriever imageRetriever;
   private final ZEDColorDepthImagePublisher imagePublisher;

   private final RDXBaseUI baseUI = new RDXBaseUI("Ball Tracker Demo");
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();
   private RDXReferenceFrameGraphic ballFrameGraphic;

   private final BallDetector ballDetector = new BallDetector();
   private final Point2f ballCenter = new Point2f(-1.0f, -1.0f);
   private final AlphaFilteredRigidBodyTransform ballPoint = new AlphaFilteredRigidBodyTransform();

   private final ImFloat positionAlpha = new ImFloat(0.1f);
   private final ImFloat[] hsvLowerBound = {new ImFloat(0.0f), new ImFloat(0.0f), new ImFloat(0.0f)};
   private final ImFloat[] hsvUpperBound = {new ImFloat(255.0f), new ImFloat(255.0f), new ImFloat(255.0f)};

   private boolean done = false;

   public RDXBallTrackingDemo()
   {
      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "ball_tracker_demo");
      ros2Helper = new ROS2Helper(ros2Node);

      imageRetriever = new ZEDColorDepthImageRetriever(0,
                                                       ReferenceFrame::getWorldFrame,
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      imagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH, PerceptionAPI.ZED2_CUT_OUT_DEPTH);

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      while (!done)
         run();

      destroy();
   }

   private void run()
   {
      ballDetector.setHSVLowerBound(hsvLowerBound[0].get(), hsvLowerBound[1].get(), hsvLowerBound[2].get());
      ballDetector.setHSVUpperBound(hsvUpperBound[0].get(), hsvUpperBound[1].get(), hsvUpperBound[2].get());

      RawImage depthImage = imageRetriever.getLatestRawDepthImage();
      RawImage leftColorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);
      RawImage rightColorImage = imageRetriever.getLatestRawColorImage(RobotSide.RIGHT);

      if (zedFocalLengthM < 0)
         zedFocalLengthM = ZED_PIXEL_SIZE * ((leftColorImage.getFocalLengthX() + leftColorImage.getFocalLengthY()) / 2.0);

      double radius = ballDetector.detect(leftColorImage, ballCenter);
      if (radius > 0.0f && ballCenter.x() > 0.0f && ballCenter.y() > 0.0f)
      {
         opencv_imgproc.circle(leftColorImage.getCpuImageMat(),
                               new Point(Math.round(ballCenter.x()), Math.round(ballCenter.y())),
                               Math.round((float) radius),
                               new Scalar(0.0, 0.0, 255.0, 255.0));
         leftColorImage.getGpuImageMat().upload(leftColorImage.getCpuImageMat());

         double depth = (BALL_DIAMETER_M * zedFocalLengthM) / (2.0 * radius * ZED_PIXEL_SIZE);
         double y = -(ballCenter.x() - leftColorImage.getPrincipalPointX()) / leftColorImage.getFocalLengthX() * depth;
         y += 0.06; // offset due to ZED lens offset from center
         double z = -(ballCenter.y() - leftColorImage.getPrincipalPointY()) / leftColorImage.getFocalLengthY() * depth;
         ballPoint.getTranslation().set(depth, y, z);
         ballPoint.setRotationToZero();

         RigidBodyTransformMessage message = new RigidBodyTransformMessage();
         MessageTools.toMessage(ballPoint, message);
         ros2Helper.publish(PerceptionAPI.BALL_TRAJECTORY, message);
      }

      imagePublisher.setNextColorImage(leftColorImage.get(), RobotSide.LEFT);
      imagePublisher.setNextColorImage(rightColorImage.get(), RobotSide.RIGHT);
      imagePublisher.setNextGpuDepthImage(depthImage.get());

      leftColorImage.release();
      depthImage.release();
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            ballFrameGraphic = new RDXReferenceFrameGraphic(0.1);

            RDXROS2ImageMessageVisualizer colorImageVisualizer = new RDXROS2ImageMessageVisualizer("Color Image",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(colorImageVisualizer, PerceptionAPI.REQUEST_ZED_COLOR);

            RDXROS2ColoredPointCloudVisualizer pointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("Point Cloud",
                                                                                                             PubSubImplementation.FAST_RTPS,
                                                                                                             PerceptionAPI.ZED2_DEPTH,
                                                                                                             PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(pointCloudVisualizer, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);

            RDXROS2TrajectoryVisualizer ballTrajectoryVisualizer = new RDXROS2TrajectoryVisualizer("Ball Trajectory", PerceptionAPI.BALL_TRAJECTORY);
            perceptionVisualizerPanel.addVisualizer(ballTrajectoryVisualizer);

            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSettings);
            baseUI.getPrimaryScene().addRenderableProvider(ballFrameGraphic);
            baseUI.create();
         }

         @Override
         public void render()
         {
            perceptionVisualizerPanel.update();
            baseUI.renderBeforeOnScreenUI();
            ballFrameGraphic.setPositionInWorldFrame(new Point3D(ballPoint.getTranslation()));
            baseUI.renderEnd();
         }

         private void renderSettings()
         {
            if (ImGui.sliderFloat("Position Alpha Filter", positionAlpha.getData(), 0.0f, 1.0f))
            {
               ballPoint.setAlpha(positionAlpha.get());
            }

            ImGui.text("HSV Lower Bound");
            ImGui.sliderFloat("Hl", hsvLowerBound[0].getData(), 0.0f, 255.0f);
            ImGui.sliderFloat("Sl", hsvLowerBound[1].getData(), 0.0f, 255.0f);
            ImGui.sliderFloat("Vl", hsvLowerBound[2].getData(), 0.0f, 255.0f);

            ImGui.text("HSV Upper Bound");
            ImGui.sliderFloat("Hu", hsvUpperBound[0].getData(), 0.0f, 255.0f);
            ImGui.sliderFloat("Su", hsvUpperBound[1].getData(), 0.0f, 255.0f);
            ImGui.sliderFloat("Vu", hsvUpperBound[2].getData(), 0.0f, 255.0f);
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

   private void destroy()
   {
      imageRetriever.destroy();
      imagePublisher.destroy();
      ballDetector.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new RDXBallTrackingDemo();
   }
}
