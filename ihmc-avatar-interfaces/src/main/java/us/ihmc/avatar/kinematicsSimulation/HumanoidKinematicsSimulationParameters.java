package us.ihmc.avatar.kinematicsSimulation;

import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

import java.nio.file.Path;

public class HumanoidKinematicsSimulationParameters
{
   private boolean createYoVariableServer = false;
   private boolean logToFile = false;
   private PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;
   private double initialGroundHeight = 0.0;
   private double initialRobotYaw = 0.0;
   private double initialRobotX = 0.0;
   private double initialRobotY = 0.0;

   public double getInitialGroundHeight()
   {
      return initialGroundHeight;
   }

   public void setInitialGroundHeight(double initialGroundHeight)
   {
      this.initialGroundHeight = initialGroundHeight;
   }

   public double getInitialRobotYaw()
   {
      return initialRobotYaw;
   }

   public void setInitialRobotYaw(double initialRobotYaw)
   {
      this.initialRobotYaw = initialRobotYaw;
   }

   public double getInitialRobotX()
   {
      return initialRobotX;
   }

   public void setInitialRobotX(double initialRobotX)
   {
      this.initialRobotX = initialRobotX;
   }

   public double getInitialRobotY()
   {
      return initialRobotY;
   }

   public void setInitialRobotY(double initialRobotY)
   {
      this.initialRobotY = initialRobotY;
   }

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
