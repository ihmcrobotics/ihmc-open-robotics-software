package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;

public class REAPerceptionSuiteElement implements PerceptionSuiteElement<LIDARBasedREAModule, LIDARBasedEnvironmentAwarenessUI>
{
   private final Stage stage;
   private final LIDARBasedREAModule perceptionModule;
   private final LIDARBasedEnvironmentAwarenessUI uiModule;

   public REAPerceptionSuiteElement(ModuleProvider<LIDARBasedREAModule> moduleProvider, UIProvider<LIDARBasedEnvironmentAwarenessUI> uiProvider) throws Exception
   {
      stage = new Stage();
      perceptionModule = moduleProvider.createModule(null);
      uiModule = uiProvider.createUI(null, stage);
      perceptionModule.start();
   }

   public LIDARBasedREAModule getPerceptionModule()
   {
      return perceptionModule;
   }

   @Override
   public LIDARBasedEnvironmentAwarenessUI getPerceptionUI()
   {
      return uiModule;
   }

   @Override
   public Stage getStage()
   {
      return stage;
   }
}
