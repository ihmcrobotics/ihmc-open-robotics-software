package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.stage.Stage;
import us.ihmc.messager.Messager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LiveMapModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.LiveMapUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LiveMapModule;

public class LiveMapPerceptionSuiteElement implements PerceptionSuiteElement<LiveMapModule, LiveMapUI>
{
   private final Messager messager;
   private final Stage stage;
   private final LiveMapModule perceptionModule;
   private final LiveMapUI uiModule;

   public LiveMapPerceptionSuiteElement(ModuleProvider<LiveMapModule> moduleProvider, UIProvider<LiveMapUI> uiProvider) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(LiveMapModuleAPI.API);
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
