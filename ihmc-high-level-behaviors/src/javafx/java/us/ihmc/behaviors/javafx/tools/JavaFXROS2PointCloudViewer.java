package us.ihmc.behaviors.javafx.tools;

import javafx.stage.Stage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.behaviors.javafx.graphics.live.LiveStereoVisionPointCloudGraphic;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.ros2.ROS2Node;

public class JavaFXROS2PointCloudViewer
{
   private final ROS2Node ros2Node;
   private LiveStereoVisionPointCloudGraphic pointCloudGraphic;

   public JavaFXROS2PointCloudViewer(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
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

      pointCloudGraphic = new LiveStereoVisionPointCloudGraphic(ros2Node, PerceptionAPI.MULTISENSE_LIDAR_SCAN);
      view3dFactory.addNodeToView(pointCloudGraphic);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      new JavaFXROS2PointCloudViewer(ROS2Tools.createInterprocessROS2Node("point_cloud_viewer"));
   }
}
