 package us.ihmc.ihmcPerception.lineSegmentDetector;
//
 import javafx.application.Application;
// import javafx.scene.Scene;
// import javafx.scene.layout.BorderPane;
// import javafx.scene.layout.Pane;
// import javafx.scene.shape.Sphere;
// import javafx.scene.transform.Translate;
 import javafx.stage.Stage;
// import us.ihmc.commons.thread.ThreadTools;
// import us.ihmc.communication.ROS2Tools;
// import us.ihmc.euclid.tuple3D.Point3D;
// import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
// import us.ihmc.javaFXToolkit.scenes.View3DFactory;
// import us.ihmc.pubsub.DomainFactory;
// // import us.ihmc.robotEnvironmentAwareness.ui.JavaFXPlanarRegionsViewer;
// import us.ihmc.ros2.ROS2Node;
//
//
 public class LineSegmentDetectorUI extends Application {

//     private Point3D sensorPosition;
//
//     private BorderPane mainPane;
//     // private JavaFXPlanarRegionsViewer planarRegionsViewer;
//     private ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");;
//
//     private LineSegmentEstimator lineSegmentEstimator;
//
     @Override
     public void start(Stage stage) throws Exception {
//         stage.setTitle("Line Segment Detector");
//         Sphere sphere = new Sphere(0.1);
//
//         mainPane = new BorderPane();
//         planarRegionsViewer = new JavaFXPlanarRegionsViewer();
//
//         lineSegmentEstimator = new LineSegmentEstimator();
//         lineSegmentEstimator.setPlanarRegionsViewer(planarRegionsViewer);
//         lineSegmentEstimator.setSensorNode(sphere);
//
//         View3DFactory view3dFactory = View3DFactory.createSubscene();
//
//
//         view3dFactory.addNodeToView(sphere);
//         view3dFactory.addNodeToView(planarRegionsViewer.getRootNode());
//         view3dFactory.addWorldCoordinateSystem(0.3);
//         Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
//         mainPane.setCenter(subScene);
//
//         FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);
//         Translate rootJointOffset = new Translate();
//         cameraController.prependTransform(rootJointOffset);
//         Translate manipulationFocusPoint = new Translate(0.0, 0.0, 0.0);
//         cameraController.prependTransform(manipulationFocusPoint);
//
//
//         Scene scene = new Scene(mainPane, 3200,1800);
//
//         stage.setOnCloseRequest(event -> stop());
//         stage.setScene(scene);
//         stage.show();
//
//         planarRegionsViewer.start();
//
     }
//
//     @Override
//     public void stop(){
//         planarRegionsViewer.stop();
//         ThreadTools.sleep(100);
//         try {
//             super.stop();
//         } catch (Exception e) {
//             e.printStackTrace();
//         }
//         System.exit(0);
//     }
 }
