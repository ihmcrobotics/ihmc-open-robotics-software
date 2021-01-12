package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.ui.GPUBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUBasedREAModule;

public class GPUBasedREAStandaloneLauncher extends Application {

    private GPUBasedEnvironmentAwarenessUI ui;
    private GPUBasedREAModule module;

    @Override
    public void start(Stage primaryStage) throws Exception {
        ui = new GPUBasedEnvironmentAwarenessUI(primaryStage);
        module = new GPUBasedREAModule();

        ui.show();
        module.start();
    }

    @Override
    public void stop() throws Exception {
        super.stop();
        ui.stop();
        module.stop();
        Platform.exit();
    }

    public static void main(String[] args)
    {
        launch(args);
    }
}
