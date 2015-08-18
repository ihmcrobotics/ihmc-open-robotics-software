package us.ihmc.valkyrie.fingers;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RevoluteJoint;

public enum ValkyrieSimulatedFingerJoint
{
   ThumbRoll, ThumbPitch1, ThumbPitch2, ThumbPitch3,
   IndexFingerPitch1, IndexFingerPitch2, IndexFingerPitch3,
   MiddleFingerPitch1, MiddleFingerPitch2, MiddleFingerPitch3,
   PinkyPitch1, PinkyPitch2, PinkyPitch3;
   
   public static final ValkyrieSimulatedFingerJoint[] values = ValkyrieSimulatedFingerJoint.values();
   
   public RevoluteJoint getRelatedRevoluteJoint(RobotSide robotSide, SDFFullRobotModel fullRobotModel)
   {
      return (RevoluteJoint) fullRobotModel.getOneDoFJointByName(getJointName(robotSide));
   }
   
   public PinJoint getRelatedPinJoint(RobotSide robotSide, SDFRobot sdfRobot)
   {
      return (PinJoint) sdfRobot.getOneDegreeOfFreedomJoint(getJointName(robotSide));
   }
   
   public String getJointName(RobotSide robotSide)
   {
      return robotSide.getCamelCaseNameForMiddleOfExpression() + name();
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
