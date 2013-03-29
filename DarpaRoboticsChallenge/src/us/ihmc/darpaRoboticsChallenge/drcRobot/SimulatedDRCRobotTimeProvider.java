package us.ihmc.darpaRoboticsChallenge.drcRobot;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;

import us.ihmc.utilities.net.TimestampProvider;

public class SimulatedDRCRobotTimeProvider implements TimestampProvider, RobotController
{
   
   private long timeStamp = 0;

   public void initialize()
   {
      timeStamp = 0;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return new YoVariableRegistry(getName());
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      timeStamp++;
   }

   public long getTimestamp()
   {
      return timeStamp;
   }

}
