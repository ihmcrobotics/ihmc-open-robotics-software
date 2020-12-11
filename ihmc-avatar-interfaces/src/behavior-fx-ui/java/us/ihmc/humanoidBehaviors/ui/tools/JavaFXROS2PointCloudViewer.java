package us.ihmc.humanoidBehaviors.ui.tools;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2TopicHz;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LiveStereoVisionPointCloudGraphic;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class JavaFXROS2PointCloudViewer
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<?> topic;
   private LiveStereoVisionPointCloudGraphic pointCloudGraphic;

   public JavaFXROS2PointCloudViewer(ROS2Node ros2Node)
   {
      this(ros2Node, ROS2Tools.D435_POINT_CLOUD);
   }

   public JavaFXROS2PointCloudViewer(ROS2Node ros2Node, ROS2Topic<?> topic)
   {
      this.ros2Node = ros2Node;
      this.topic = topic;

      JavaFXApplicationCreator.buildJavaFXApplication(this::build);
   }

   private void build(Stage primaryStage)
   {
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      FocusBasedCameraMouseEventHandler camera = view3dFactory.addCameraController(0.05, 2000.0, true);
      double isoZoomOut = 10.0;
      camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      pointCloudGraphic = new LiveStereoVisionPointCloudGraphic(ros2Node, topic);
      view3dFactory.addNodeToView(pointCloudGraphic);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      new JavaFXROS2PointCloudViewer(ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "point_cloud_viewer"));
   }
}
