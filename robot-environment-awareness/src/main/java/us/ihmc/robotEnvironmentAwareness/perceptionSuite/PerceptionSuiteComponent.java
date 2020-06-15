package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;

import java.util.ArrayList;
import java.util.List;

public class PerceptionSuiteComponent<M extends PerceptionModule, U extends PerceptionUI>
{
   private final Topic<Boolean> runModuleTopic;
   private final Topic<Boolean> runUITopic;

   private Stage uiStage;
   private M module;
   private U ui;

   private final ModuleProvider<M> moduleProvider;
   private final UIProvider<U> uiProvider;
   private final String name;
   private final Messager messager;

   private final List<PerceptionSuiteComponent<?, ?>> dependentModules = new ArrayList<>();

   public PerceptionSuiteComponent(String name,
                                   ModuleProvider<M> moduleProvider,
                                   UIProvider<U> uiProvider,
                                   Messager messager,
                                   Topic<Boolean> runModuleTopic,
                                   Topic<Boolean> runUITopic)
   {
      this.name = name;
      this.moduleProvider = moduleProvider;
      this.uiProvider = uiProvider;
      this.messager = messager;
      this.runModuleTopic = runModuleTopic;
      this.runUITopic = runUITopic;

      messager.registerTopicListener(runModuleTopic, run -> run(run, this::startModule, this::stopModule));
      messager.registerTopicListener(runUITopic, run -> run(run, this::startUI, this::stopUI));
   }

   public void attachDependentModule(PerceptionSuiteComponent<?, ?> dependentModule)
   {
      this.dependentModules.add(dependentModule);
   }

   public M getModule()
   {
      return module;
   }

   public String getName()
   {
      return name;
   }

   public void startModule() throws Exception
   {
      if (module == null)
      {
         module = moduleProvider.createModule();
         module.start();
      }
      else
      {
         throw new RuntimeException(name + " is already running.");
      }
   }

   public void startUI()
   {
      if (module == null)
      {
         String error = name + " Module must be running first.";
         LogTools.error(error);
         messager.submitMessage(PerceptionSuiteAPI.ErrorMessage, error);
         messager.submitMessage(runUITopic, false);
         return;
      }

      if (ui == null)
      {
         Platform.runLater(() ->
                           {
                              uiStage = new Stage();
                              try
                              {
                                 ui = uiProvider.createUI(uiStage);
                                 ui.show();
                              }
                              catch (Exception e)
                              {
                                 LogTools.warn(e.getMessage());
                                 messager.submitMessage(PerceptionSuiteAPI.ErrorMessage, e.getMessage());
                              }
                              uiStage.setOnCloseRequest(event ->
                                                        {
                                                           messager.submitMessage(runUITopic, false);
                                                           stopUI();
                                                        });
                           });
      }
      else
      {
         stopUI();
         String error = name + " UI is already running.";
         LogTools.error(error);
         messager.submitMessage(PerceptionSuiteAPI.ErrorMessage, error);
      }
   }

   public void stopModule()
   {
      if (module != null)
      {
         module.stop();
         module = null;
      }

      stopUI();
      messager.submitMessage(runModuleTopic, false);

      for (PerceptionSuiteComponent<?, ?> dependentModule : dependentModules)
      {
         dependentModule.stopUI();
         dependentModule.stopModule();
      }
   }

   public void stopUI()
   {
      if (ui != null)
      {
         Platform.runLater(() ->
                           {
                              uiStage.close();
                              ui.stop();
                              uiStage = null;
                              ui = null;
                           });
      }
      messager.submitMessage(runUITopic, false);
   }

   public void stop()
   {
      stopUI();
      stopModule();
   }

   private interface Command
   {
      void run() throws Exception;
   }

   private void run(boolean run, Command start, Command stop)
   {
      if (run)
      {
         try
         {
            start.run();
         }
         catch (Exception e)
         {
            String error = "Failed to start " + name + ": " + e.getMessage();
            LogTools.error(error);
            messager.submitMessage(PerceptionSuiteAPI.ErrorMessage, error);
         }
      }
      else
      {
         try
         {
            stop.run();
         }
         catch (Exception e)
         {
            String error = "Failed to stop " + name + ": " + e.getMessage();
            LogTools.error(error);
            messager.submitMessage(PerceptionSuiteAPI.ErrorMessage, error);
         }
      }
   }

   interface ModuleProvider<T>
   {
      T createModule() throws Exception;
   }

   interface UIProvider<T>
   {
      T createUI(Stage stage) throws Exception;
   }
}
