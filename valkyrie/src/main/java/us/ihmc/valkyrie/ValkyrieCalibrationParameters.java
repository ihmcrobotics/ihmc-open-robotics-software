package us.ihmc.valkyrie;

import java.util.HashMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieCalibrationParameters implements WholeBodySetpointParameters
{
   private final HashMap<String, Double> setPoints = new HashMap<>();
   private final ValkyrieJointMap jointMap;

   public ValkyrieCalibrationParameters(ValkyrieJointMap jointMap)
   {
      this.jointMap = jointMap;
      useDefaultAngles();
   }

   private void useDefaultAngles()
   {
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), -0.06);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.25);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 0.7);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 0.0);

         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.3);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfLeftSide(1.1708));
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.05);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), robotSide.negateIfLeftSide(1.7));

         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), 0.5);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
      }

      // Not useful for calibration itself, but helps checking that the neck J3 is working.
      setSetpoint(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.25);
      setSetpoint(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), -0.25);
   }

   private void setSetpoint(String jointName, double value)
   {
      setPoints.put(jointName, value);
   }

   @Override
   public double getSetpoint(String jointName)
   {
      if (setPoints.containsKey(jointName))
         return setPoints.get(jointName);
      else
         return 0.0;
   }
}
