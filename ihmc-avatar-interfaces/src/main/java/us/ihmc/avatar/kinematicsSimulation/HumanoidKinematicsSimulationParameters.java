package us.ihmc.avatar.kinematicsSimulation;

import us.ihmc.commons.UnitConversions;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * To be used with {@link HumanoidKinematicsSimulation}.
 */
public class HumanoidKinematicsSimulationParameters
{
   private boolean createPeriodicThread = true;
   private boolean createYoVariableServer = false;
   private boolean logToFile = false;
   private double initialGroundHeight = 0.0;
   private double initialRobotYaw = 0.0;
   private double initialRobotX = 0.0;
   private double initialRobotY = 0.0;
   private double initialRobotZ = 0.0;
   private double playbackSpeedMultiplier = 10.0;
   private double dt = UnitConversions.hertzToSeconds(70);
   private boolean runNoFasterThanMaxRealtimeRate = true;
   private double maxRealtimeRate = 2.0;
   private ROS2Node ros2Node = null;
   private final List<YoRegistry> registriesToInclude = new ArrayList<>();

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

   public double getInitialRobotZ()
   {
      return initialRobotZ;
   }

   public void setInitialRobotZ(double initialRobotZ)
   {
      this.initialRobotZ = initialRobotZ;
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

   public void setPlaybackSpeedMultiplier(double playbackSpeedMultiplier)
   {
      this.playbackSpeedMultiplier = playbackSpeedMultiplier;
   }

   public void setUpdateFrequencyHz(double updateFrequencyHz)
   {
      this.dt = UnitConversions.hertzToSeconds(updateFrequencyHz);
   }

   public double getDt()
   {
      return dt;
   }

   public double getUpdatePeriod()
   {
      return dt / playbackSpeedMultiplier;
   }

   public void setRunNoFasterThanMaxRealtimeRate(boolean runNoFasterThanMaxRealtimeRate)
   {
      this.runNoFasterThanMaxRealtimeRate = runNoFasterThanMaxRealtimeRate;
   }

   public boolean runNoFasterThanMaxRealtimeRate()
   {
      return runNoFasterThanMaxRealtimeRate;
   }

   public void setMaxRealtimeRate(double maxRealtimeRate)
   {
      this.maxRealtimeRate = maxRealtimeRate;
   }

   public double getMaxRealtimeRate()
   {
      return maxRealtimeRate;
   }

   public boolean isPeriodicThreadEnabled()
   {
      return createPeriodicThread;
   }

   public void setEnablePeriodicThread(boolean createPeriodicThread)
   {
      this.createPeriodicThread = createPeriodicThread;
   }

   public void setROS2Node(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public ROS2Node getROS2Node()
   {
      return ros2Node;
   }

   public List<YoRegistry> getRegistriesToInclude()
   {
      return registriesToInclude;
   }
}
