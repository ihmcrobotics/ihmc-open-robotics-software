package us.ihmc.valkyrie.fingers;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.PinJoint;

public enum ValkyrieSimulatedFingerJoint
{
   ThumbRoll, ThumbPitch1, ThumbPitch2, ThumbPitch3,
   IndexFingerPitch1, IndexFingerPitch2, IndexFingerPitch3,
   MiddleFingerPitch1, MiddleFingerPitch2, MiddleFingerPitch3,
   PinkyPitch1, PinkyPitch2, PinkyPitch3;

   public static final ValkyrieSimulatedFingerJoint[] values = ValkyrieSimulatedFingerJoint.values();

   public RevoluteJoint getRelatedRevoluteJoint(RobotSide robotSide, FullRobotModel fullRobotModel)
   {
      return (RevoluteJoint) fullRobotModel.getOneDoFJointByName(getJointName(robotSide));
   }

   public PinJoint getRelatedPinJoint(RobotSide robotSide, FloatingRootJointRobot sdfRobot)
   {
      return (PinJoint) sdfRobot.getOneDegreeOfFreedomJoint(getJointName(robotSide));
   }

   public String getJointName(RobotSide robotSide)
   {
      return robotSide.getCamelCaseNameForStartOfExpression() + name();
   }

   public ValkyrieRealRobotFingerJoint getRelatedRealFingerJoint()
   {
      switch (this)
      {
      case ThumbRoll:
         return ValkyrieRealRobotFingerJoint.ThumbRoll;
      case ThumbPitch1:
      case ThumbPitch2:
      case ThumbPitch3:
         return ValkyrieRealRobotFingerJoint.Thumb;
      case IndexFingerPitch1:
      case IndexFingerPitch2:
      case IndexFingerPitch3:
         return ValkyrieRealRobotFingerJoint.Index;
      case MiddleFingerPitch1:
      case MiddleFingerPitch2:
      case MiddleFingerPitch3:
         return ValkyrieRealRobotFingerJoint.Middle;
      case PinkyPitch1:
      case PinkyPitch2:
      case PinkyPitch3:
         return ValkyrieRealRobotFingerJoint.Pinky;
      default:
         return null;
      }
   }
}
