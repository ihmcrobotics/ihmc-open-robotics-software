package us.ihmc.valkyrie.fingers;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.PinJoint;

public enum ValkyrieHandJointName implements HandJointName
{
   ThumbRoll,
   ThumbPitch1,
   ThumbPitch2,
   ThumbPitch3,
   IndexFingerPitch1,
   IndexFingerPitch2,
   IndexFingerPitch3,
   MiddleFingerPitch1,
   MiddleFingerPitch2,
   MiddleFingerPitch3,
   PinkyPitch1,
   PinkyPitch2,
   PinkyPitch3;

   public static final ValkyrieHandJointName[] values = values();

   @Override
   public String getJointName(RobotSide robotSide)
   {
      return getCamelCaseJointName(robotSide);
   }

   public String getCamelCaseJointName(RobotSide side)
   {
      return side.getCamelCaseName() + name();
   }

   public String getPascalCaseJointName(RobotSide side)
   {
      return side.getPascalCaseName() + name();
   }

   public FingerName getFinger(RobotSide robotSide)
   {
      return getFingerName();
   }

   public FingerName getFingerName()
   {
      switch (this)
      {
      case ThumbRoll:
      case ThumbPitch1:
      case ThumbPitch2:
      case ThumbPitch3:
         return FingerName.THUMB;
      case IndexFingerPitch1:
      case IndexFingerPitch2:
      case IndexFingerPitch3:
         return FingerName.INDEX;
      case MiddleFingerPitch1:
      case MiddleFingerPitch2:
      case MiddleFingerPitch3:
         return FingerName.MIDDLE;
      case PinkyPitch1:
      case PinkyPitch2:
      case PinkyPitch3:
         return FingerName.PINKY;
      default:
         throw new RuntimeException("Unexpected " + getClass().getSimpleName() + " value: " + this);
      }
   }

   public static int getNumberOfFingerJoints(FingerName fingerName)
   {
      int numberOfFingerJoints = 0;
      for (ValkyrieHandJointName joint : values)
      {
         if (joint.getFingerName() == fingerName)
            numberOfFingerJoints++;
      }
      return numberOfFingerJoints;
   }

   @Override
   public int getIndex(RobotSide robotSide)
   {
      return ordinal();
   }

   public RevoluteJoint getRelatedRevoluteJoint(RobotSide robotSide, FullRobotModel fullRobotModel)
   {
      return (RevoluteJoint) fullRobotModel.getOneDoFJointByName(getJointName(robotSide));
   }

   public PinJoint getRelatedPinJoint(RobotSide robotSide, FloatingRootJointRobot sdfRobot)
   {
      return (PinJoint) sdfRobot.getOneDegreeOfFreedomJoint(getJointName(robotSide));
   }
}
