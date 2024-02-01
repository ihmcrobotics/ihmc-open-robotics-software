package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.stage.Stage;
import us.ihmc.messager.Messager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;

public class SegmentationPerceptionSuiteElement implements PerceptionSuiteElement<PlanarSegmentationModule, PlanarSegmentationUI>
{
   private final Messager messager;
   private final Stage stage;
   private final PlanarSegmentationModule perceptionModule;
   private final PlanarSegmentationUI uiModule;

   public SegmentationPerceptionSuiteElement(ModuleProvider<PlanarSegmentationModule> moduleProvider, UIProvider<PlanarSegmentationUI> uiProvider) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(SegmentationModuleAPI.API);
      messager.startMessager();

      stage = new Stage();
      perceptionModule = moduleProvider.createModule(messager);
      uiModule = uiProvider.createUI(messager, stage);
      perceptionModule.start();
   }

   @Override
   public void stopInternal()
   {
      try
      {
         if (messager.isMessagerOpen())
            messager.closeMessager();
      }
      catch (Exception e)
      {

      }
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
