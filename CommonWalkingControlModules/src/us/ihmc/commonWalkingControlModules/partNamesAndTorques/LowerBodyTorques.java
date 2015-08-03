package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.robotics.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.robotSide.RobotSide;

public class LowerBodyTorques
{
   private LegTorques[] legTorquesArray = new LegTorques[RobotSide.values.length];

   public LowerBodyTorques(RobotSpecificJointNames robotJointNames)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         legTorquesArray[robotSide.ordinal()] = new LegTorques(robotJointNames, robotSide);
      }
   }

// private LowerBodyTorques(LowerBodyTorques lowerBodyTorques)
// {
//    for (RobotSide robotSide : RobotSide.values)
//    {
//       legTorquesArray[robotSide.ordinal()] = lowerBodyTorques.getLegTorquesCopy(robotSide);
//    }
// }

   public LegTorques getLegTorques(RobotSide robotSide)
   {
      return legTorquesArray[robotSide.ordinal()];
   }

   public LegTorques getLegTorquesCopy(RobotSide robotSide)
   {
      return legTorquesArray[robotSide.ordinal()].getLegTorquesCopy();
   }

   public void copyLegTorquesValuesToLowerBodyTorques(LegTorques legTorques)
   {
      legTorquesArray[legTorques.getRobotSide().ordinal()] = legTorques.getLegTorquesCopy();
   }

   public void setLowerBodyTorquesToZero()
   {
      for (int i = 0; i < legTorquesArray.length; i++)
      {
         legTorquesArray[i].setTorquesToZero();
      }
   }
}
