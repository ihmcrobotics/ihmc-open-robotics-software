package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.ui.LiveMapUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LiveMapModule;

public class LiveMapPerceptionSuiteElement implements PerceptionSuiteElement<LiveMapModule, LiveMapUI>
{
   private final Stage stage;
   private final LiveMapModule perceptionModule;
   private final LiveMapUI uiModule;

   public LiveMapPerceptionSuiteElement(ModuleProvider<LiveMapModule> moduleProvider, UIProvider<LiveMapUI> uiProvider) throws Exception
   {
      stage = new Stage();
      perceptionModule = moduleProvider.createModule();
      uiModule = uiProvider.createUI(stage);

      perceptionModule.start();

      stage.setOnCloseRequest((event) -> hide());
   }

   public LiveMapModule getPerceptionModule()
   {
      return perceptionModule;
   }

   @Override
   public LiveMapUI getPerceptionUI()
   {
      return uiModule;
   }

   @Override
   public Stage getStage()
   {
      return stage;
   }
}
