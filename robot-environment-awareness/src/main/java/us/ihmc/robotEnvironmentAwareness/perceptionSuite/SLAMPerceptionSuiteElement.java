package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;

public class SLAMPerceptionSuiteElement implements PerceptionSuiteElement<SLAMModule, SLAMBasedEnvironmentAwarenessUI>
{
   private final Stage stage;
   private final SLAMModule perceptionModule;
   private final SLAMBasedEnvironmentAwarenessUI uiModule;

   public SLAMPerceptionSuiteElement(ModuleProvider<SLAMModule> moduleProvider, UIProvider<SLAMBasedEnvironmentAwarenessUI> uiProvider) throws Exception
   {
      stage = new Stage();
      perceptionModule = moduleProvider.createModule(null);
      uiModule = uiProvider.createUI(null, stage);

      perceptionModule.start();

      stage.setOnCloseRequest((event) -> hide());
   }

   public SLAMModule getPerceptionModule()
   {
      return perceptionModule;
   }

   @Override
   public SLAMBasedEnvironmentAwarenessUI getPerceptionUI()
   {
      return uiModule;
   }

   @Override
   public Stage getStage()
   {
      return stage;
   }
}
