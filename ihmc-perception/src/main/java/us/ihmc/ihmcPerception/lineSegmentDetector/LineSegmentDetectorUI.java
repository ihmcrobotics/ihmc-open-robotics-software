package us.ihmc.ihmcPerception.lineSegmentDetector;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.ui.JavaFXPlanarRegionsViewer;
import us.ihmc.ros2.Ros2Node;


public class LineSegmentDetectorUI extends Application {

    private BorderPane mainPane;
    private JavaFXPlanarRegionsViewer planarRegionsViewer = new JavaFXPlanarRegionsViewer();
    private Ros2Node ros2Node;

    private LineSegmentEstimator lineSegmentEstimator;


    @Override
    public void start(Stage stage) throws Exception {
        stage.setTitle("Line Segment Detector");


        Button button = new Button("Click Me");

        mainPane = new BorderPane();
        mainPane.getChildren().add(button);

        this.ros2Node = ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
        ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.outputTopic,
                s -> planarRegionsViewer.submitPlanarRegions(s.takeNextData()));

        View3DFactory view3dFactory = View3DFactory.createSubscene();

        Sphere sphere = new Sphere(0.1);

        view3dFactory.addNodeToView(sphere);


        view3dFactory.addNodeToView(planarRegionsViewer.getRootNode());

        view3dFactory.addWorldCoordinateSystem(0.3);
        Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
        mainPane.setCenter(subScene);

        FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);

        Translate rootJointOffset = new Translate();
        cameraController.prependTransform(rootJointOffset);
        Translate manipulationFocusPoint = new Translate(0.0, 0.0, 1.0);
        cameraController.prependTransform(manipulationFocusPoint);

        this.planarRegionsViewer.start();

        Scene scene = new Scene(mainPane, 3200,1800);

        stage.setScene(scene);
        stage.show();


        // this.lineSegmentEstimator = new LineSegmentEstimator();

        // this.planarRegionsViewer.submitPlanarRegions(currentPlanarRegionsListMessage);


    }
}
