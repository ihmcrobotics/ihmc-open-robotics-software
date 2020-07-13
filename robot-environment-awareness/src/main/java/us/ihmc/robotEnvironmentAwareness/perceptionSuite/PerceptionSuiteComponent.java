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
   private final Topic<Boolean> showUITopic;

   private PerceptionSuiteElement<M, U> element;

   private final ElementProvider<M, U> elementProvider;
   private final String name;
   private final Messager messager;

   private final List<PerceptionSuiteComponent<?, ?>> dependentModules = new ArrayList<>();

   public PerceptionSuiteComponent(String name,
                                   ElementProvider<M, U> elementProvider,
                                   Messager messager,
                                   Topic<Boolean> runModuleTopic,
                                   Topic<Boolean> showUITopic)
   {
      this.name = name;
      this.elementProvider = elementProvider;
      this.messager = messager;
      this.runModuleTopic = runModuleTopic;
      this.showUITopic = showUITopic;

      messager.registerTopicListener(runModuleTopic, run -> run(run, this::startModule, this::stopModule));
      messager.registerTopicListener(showUITopic, run -> run(run, this::showUI, this::hideUI));
   }

   public void attachDependentModule(PerceptionSuiteComponent<?, ?> dependentModule)
   {
      this.dependentModules.add(dependentModule);
   }

   public PerceptionSuiteElement<M, U> getElement()
   {
      return element;
   }

   public String getName()
   {
      return name;
   }

   public void startModule()
   {
      if (element == null)
      {


         Platform.runLater(() ->
                           {
                              try
                              {
                                 element = elementProvider.createElement();
                              }
                              catch (Exception e)
                              {

                              }
                           });
      }
      else
      {
         throw new RuntimeException(name + " is already running.");
      }
   }

   public void showUI()
   {
      if (element == null)
      {
         String error = "UI " + name + " is not running.";
         LogTools.error(error);
         messager.submitMessage(PerceptionSuiteAPI.ErrorMessage, error);
         return;
      }

      Platform.runLater(() ->
                        {
                           element.show();
                        });
   }

   public void stopModule()
   {
      if (element != null)
      {
         element.stop();
         element = null;
      }

      messager.submitMessage(runModuleTopic, false);
      messager.submitMessage(showUITopic, false);

      for (PerceptionSuiteComponent<?, ?> dependentModule : dependentModules)
      {
         dependentModule.stopModule();
      }
   }

   public void hideUI()
   {
      Platform.runLater(() ->
                        {
                           element.hide();
                           messager.submitMessage(showUITopic, false);
                        });
   }

   public void stop()
   {
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
            e.printStackTrace();
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

   interface ElementProvider<M extends PerceptionModule, U extends PerceptionUI>
   {
      PerceptionSuiteElement<M, U> createElement() throws Exception;
   }

}
