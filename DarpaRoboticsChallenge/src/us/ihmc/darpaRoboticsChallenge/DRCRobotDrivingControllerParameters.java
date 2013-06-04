package us.ihmc.darpaRoboticsChallenge;


import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import java.util.LinkedHashMap;
import java.util.Map;

public class DRCRobotDrivingControllerParameters extends DRCRobotWalkingControllerParameters
{
   @Override
   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();

      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-1.36));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH), 0.34);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(1.5));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH), 1.94);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_PITCH), -0.19);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_ROLL), robotSide.negateIfRightSide(-0.07));

      return jointPositions;
   }


   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
      return new String[] {"back_lbz", "neck_ay"};
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      return new String[] {"back_ubx", "back_mby"};
   }

   @Override
   public String getJointNameForExtendedPitchRange()
   {
      return null;
   }

}
