package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;

public class DummyPerceptionSuiteComponent
{
   private final String name;
   private final Messager messager;

   public DummyPerceptionSuiteComponent(String name,
                                        Messager messager,
                                        Topic<Boolean> runModuleTopic,
                                        Topic<Boolean> showUITopic)
   {
      this.name = name;
      this.messager = messager;

      messager.registerTopicListener(runModuleTopic, this::run);
      messager.registerTopicListener(showUITopic, this::run);
   }


   public String getName()
   {
      return name;
   }

   public void stop()
   {
   }

   private void run(boolean run)
   {
      if (run)
      {
         String error = "Connot run " + name + ".";
         LogTools.error(error);
         messager.submitMessage(PerceptionSuiteAPI.ErrorMessage, error);
      }
   }
}
