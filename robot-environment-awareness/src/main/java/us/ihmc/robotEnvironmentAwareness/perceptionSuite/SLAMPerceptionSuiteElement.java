package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.stage.Stage;
import us.ihmc.messager.Messager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;

public class SLAMPerceptionSuiteElement implements PerceptionSuiteElement<SLAMModule, SLAMBasedEnvironmentAwarenessUI>
{
   private final Messager messager;
   private final Stage stage;
   private final SLAMModule perceptionModule;
   private final SLAMBasedEnvironmentAwarenessUI uiModule;

   public SLAMPerceptionSuiteElement(ModuleProvider<SLAMModule> moduleProvider, UIProvider<SLAMBasedEnvironmentAwarenessUI> uiProvider) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(SLAMModuleAPI.API);
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

   @Override
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
