package us.ihmc.valkyrie.fingers;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public class ValkyrieHandModel implements HandModel
{
   private static HandJointName[] handJointNames = new HandJointName[ValkyrieFingerJoint.values.length];

   static
   {
      for (int i = 0; i < ValkyrieFingerJoint.values.length; i++)
      {
         ValkyrieFingerJoint jointName = ValkyrieFingerJoint.values[i];
         
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
               return jointName.getJointName(robotSide);
            }

            @Override
            public int getHandJointAngleIndex()
            {
               return jointName.ordinal();
            }

            @Override
            public FingerName getFinger(RobotSide robotSide)
            {
               return jointName.getFingerName();
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
