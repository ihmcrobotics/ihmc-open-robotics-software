package us.ihmc.valkyrie.fingers;

import us.ihmc.communication.packets.dataobjects.HandJointName;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.utilities.humanoidRobot.partNames.FingerName;
import us.ihmc.utilities.robotSide.RobotSide;

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
               switch (ValkyrieSimulatedFingerJoint.values[finalIndex])
               {
               case ThumbRoll:
               case ThumbPitch1:
               case ThumbPitch2:
               case ThumbPitch3:
                  return FingerName.INDEX;
               case IndexFingerPitch1:
               case IndexFingerPitch2:
               case IndexFingerPitch3:
                  return FingerName.INDEX;
               case MiddleFingerPitch1:
               case MiddleFingerPitch2:
               case MiddleFingerPitch3:
                  return FingerName.INDEX;
               case PinkyPitch1:
               case PinkyPitch2:
               case PinkyPitch3:
                  return FingerName.INDEX;
               default:
                  return null;
               }
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
