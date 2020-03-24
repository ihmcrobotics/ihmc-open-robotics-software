package us.ihmc.avatar.kinematicsSimulation;

import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

import java.nio.file.Path;

public class HumanoidKinematicsSimulationParameters
{
   private boolean createYoVariableServer = false;
   private boolean logToFile = false;
   private PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;
   private Path incomingLogsDirectory;

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

   public void setIncomingLogsDirectory(Path incomingLogsDirectory)
   {
      this.incomingLogsDirectory = incomingLogsDirectory;
   }

   public Path getIncomingLogsDirectory()
   {
      return incomingLogsDirectory;
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
