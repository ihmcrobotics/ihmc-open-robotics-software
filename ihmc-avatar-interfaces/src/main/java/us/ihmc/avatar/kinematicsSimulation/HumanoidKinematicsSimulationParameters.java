package us.ihmc.avatar.kinematicsSimulation;

import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class HumanoidKinematicsSimulationParameters
{
   private boolean createYoVariableServer = false;
   private boolean logToFile = false;
   private PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;

   public void setCreateYoVariableServer(boolean createYoVariableServer)
   {
      this.createYoVariableServer = createYoVariableServer;
   }

   public boolean getCreateYoVariableServer()
   {
      return createYoVariableServer;
   }

   public void setLogToFile(boolean logToFile)
   {
      this.logToFile = logToFile;
   }

   public boolean getLogToFile()
   {
      return logToFile;
   }

   public void setPubSubImplementation(PubSubImplementation pubSubImplementation)
   {
      this.pubSubImplementation = pubSubImplementation;
   }

   public PubSubImplementation getPubSubImplementation()
   {
      return pubSubImplementation;
   }
}
