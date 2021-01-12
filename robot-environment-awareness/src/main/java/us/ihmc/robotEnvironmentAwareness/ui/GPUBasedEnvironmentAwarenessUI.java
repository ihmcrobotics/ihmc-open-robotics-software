package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionUI;

public class GPUBasedEnvironmentAwarenessUI implements PerceptionUI {

    private final BorderPane mainPane;
    private final Stage primaryStage;

    public GPUBasedEnvironmentAwarenessUI(Stage primaryStage) throws Exception {
        this.primaryStage = primaryStage;
        FXMLLoader loader = new FXMLLoader();
        loader.setController(this);
        loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
        mainPane = loader.load();

        View3DFactory view3dFactory = View3DFactory.createSubscene();
        view3dFactory.addCameraController(true);
        view3dFactory.addWorldCoordinateSystem(0.3);
        mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

        primaryStage.setTitle(getClass().getSimpleName());
        primaryStage.setMaximized(true);
        Scene mainScene = new Scene(mainPane, 600, 400);

        primaryStage.setScene(mainScene);
        primaryStage.setOnCloseRequest(event -> stop());
    }

    @Override
    public void show() {
        primaryStage.show();
    }

    @Override
    public void stop() {

    }
}
