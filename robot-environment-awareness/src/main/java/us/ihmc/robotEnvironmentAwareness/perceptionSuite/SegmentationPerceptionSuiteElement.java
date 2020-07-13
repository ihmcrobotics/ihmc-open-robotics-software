package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;

public class SegmentationPerceptionSuiteElement implements PerceptionSuiteElement<PlanarSegmentationModule, PlanarSegmentationUI>
{
   private final Stage stage;
   private final PlanarSegmentationModule perceptionModule;
   private final PlanarSegmentationUI uiModule;

   public SegmentationPerceptionSuiteElement(ModuleProvider<PlanarSegmentationModule> moduleProvider, UIProvider<PlanarSegmentationUI> uiProvider) throws Exception
   {
      stage = new Stage();
      perceptionModule = moduleProvider.createModule();
      uiModule = uiProvider.createUI(stage);
      perceptionModule.start();

      stage.setOnCloseRequest((event) -> hide());
   }

   public PlanarSegmentationModule getPerceptionModule()
   {
      return perceptionModule;
   }

   @Override
   public PlanarSegmentationUI getPerceptionUI()
   {
      return uiModule;
   }

   @Override
   public Stage getStage()
   {
      return stage;
   }
}
