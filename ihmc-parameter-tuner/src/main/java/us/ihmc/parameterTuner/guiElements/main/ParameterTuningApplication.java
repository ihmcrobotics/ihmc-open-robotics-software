package us.ihmc.parameterTuner.guiElements.main;

import java.util.List;

import javafx.animation.AnimationTimer;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.parameterTuner.JavaFXExceptionTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;

public abstract class ParameterTuningApplication extends ApplicationNoModule
{
   private static final String FXML_FILE = "/gui.fxml";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.getIcons().add(new Image(ParameterTuningApplication.class.getResourceAsStream("/icon.png")));

      ParameterGuiInterface guiInterface = createInputManager();

      FXMLLoader mainLoader = new FXMLLoader();
      mainLoader.setLocation(ParameterTuningApplication.class.getResource(FXML_FILE));
      Scene mainScene = new Scene(mainLoader.<Pane> load());

      GuiController controller = mainLoader.getController();
      controller.addInputNode(guiInterface.getInputManagerNode());

      AnimationTimer animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long timestamp)
         {
            // Check if registry structure needs to be reloaded.
            if (guiInterface.pollReloadAll())
            {
               List<GuiRegistry> fullRegistries = guiInterface.getRegistriesCopy();
               controller.setRegistries(fullRegistries);
            }

            // Check if the user changed the root registries.
            if (controller.areRootRegistriesChanged())
            {
               guiInterface.changeRootRegistries(controller.pollRootRegistryNames());
            }

            // If parameters were changed in the GUI forward copies to the interface.
            List<GuiParameter> changedParameters = controller.pollChangedParameters();
            if (changedParameters != null && !changedParameters.isEmpty())
            {
               guiInterface.submitChangedParameters(changedParameters);
            }

            // If parameters were changed externally update the internal parameters.
            List<GuiParameter> updatedParameters = guiInterface.pollUpdatedParameters();
            if (updatedParameters != null && !updatedParameters.isEmpty())
            {
               controller.updateParameters(updatedParameters);
            }
         }
      };

      animationTimer.start();
      primaryStage.setOnCloseRequest(event -> {
         animationTimer.stop();
         guiInterface.shutdown();
         controller.close();
      });

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setScene(mainScene);
      primaryStage.setHeight(800.0);
      primaryStage.setWidth(1400.0);

      JavaFXExceptionTools.setupExceptionHandling();

      primaryStage.show();
   }

   protected abstract ParameterGuiInterface createInputManager();
}
