package us.ihmc.valkyrie.fingers;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public class ValkyrieHandModel implements HandModel
{
   private static HandJointName[] handJointNames = new HandJointName[ValkyrieSimulatedFingerJoint.values.length];

   static
   {
      for (int i = 0; i < ValkyrieSimulatedFingerJoint.values.length; i++)
      {
         final int finalIndex = i;
         handJointNames[i] = new HandJointName()
         {
            @Override
            public HandJointName[] getValues()
            {
               return handJointNames;
            }

            @Override
            public String getJointName(RobotSide robotSide)
            {
               return ValkyrieSimulatedFingerJoint.values[finalIndex].getJointName(robotSide);
            }

            @Override
            public int getHandJointAngleIndex()
            {
               return finalIndex;
            }

            @Override
            public FingerName getFinger(RobotSide robotSide)
            {
               return FingerName.INDEX;
            }
         };
      }
   }

   @Override
   public HandJointName[] getHandJointNames()
   {
      return handJointNames;
   }
}
