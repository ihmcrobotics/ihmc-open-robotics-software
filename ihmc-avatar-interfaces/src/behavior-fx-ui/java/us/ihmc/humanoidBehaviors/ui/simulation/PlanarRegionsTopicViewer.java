package us.ihmc.humanoidBehaviors.ui.simulation;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.ui.graphics.live.JavaFXLivePlanarRegionsGraphic;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;

public class PlanarRegionsTopicViewer extends Application
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "planar_region_topic_viewer");
      
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      FocusBasedCameraMouseEventHandler camera = view3dFactory.addCameraController(0.05, 2000.0, true);
      double isoZoomOut = 10.0;
      camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      JavaFXLivePlanarRegionsGraphic regionsGraphic = new JavaFXLivePlanarRegionsGraphic(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS, false);
      view3dFactory.addNodeToView(regionsGraphic);

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
