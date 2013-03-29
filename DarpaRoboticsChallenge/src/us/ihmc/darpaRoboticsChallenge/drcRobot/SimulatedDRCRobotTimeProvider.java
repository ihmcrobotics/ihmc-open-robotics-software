package us.ihmc.darpaRoboticsChallenge.drcRobot;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;

import us.ihmc.utilities.net.TimeStampProvider;

public class SimulatedDRCRobotTimeProvider implements TimeStampProvider, RobotController
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

   public long getTimeStamp()
   {
      return timeStamp;
   }

}
