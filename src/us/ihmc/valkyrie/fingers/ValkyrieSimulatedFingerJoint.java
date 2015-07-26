package us.ihmc.valkyrie.fingers;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public enum ValkyrieSimulatedFingerJoint
{
   ThumbRoll, ThumbPitch1, ThumbPitch2, ThumbPitch3,
   IndexFingerPitch1, IndexFingerPitch2, IndexFingerPitch3,
   MiddleFingerPitch1, MiddleFingerPitch2, MiddleFingerPitch3,
   PinkyPitch1, PinkyPitch2, PinkyPitch3;
   
   public static final ValkyrieSimulatedFingerJoint[] values = ValkyrieSimulatedFingerJoint.values();
   
   public OneDoFJoint getRelatedOneDofJoint(RobotSide robotSide, SDFFullRobotModel fullRobotModel)
   {
      return fullRobotModel.getOneDoFJointByName(getJointName(robotSide));
   }
   
   public String getJointName(RobotSide robotSide)
   {
      return robotSide.getCamelCaseNameForMiddleOfExpression() + name();
   }
}
