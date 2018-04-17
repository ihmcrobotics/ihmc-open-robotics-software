package us.ihmc.valkyrie;

import java.util.HashMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.PositionControlParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;

public class ValkyrieStandPrepParameters implements WholeBodySetpointParameters, PositionControlParameters
{
   private final HashMap<String, Double> setPoints = new HashMap<>();
   private final HashMap<String, Double> proportionalGain = new HashMap<>();
   private final HashMap<String, Double> integralGain = new HashMap<>();
   private final HashMap<String, Double> derivativeGain = new HashMap<>();
   private final ValkyrieJointMap jointMap;

   public ValkyrieStandPrepParameters(ValkyrieJointMap jointMap)
   {
      this.jointMap = jointMap;
      useDefaultAngles();
      useDefaultGains();
   }

   private void useDefaultAngles()
   {
      setSetpoint(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.0);
      setSetpoint(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0);
      setSetpoint(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), 0.0);

      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.6);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 1.3);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.65);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 0.0);

         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), -0.2);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfLeftSide(1.2));
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), robotSide.negateIfLeftSide(1.5));
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), 1.3);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
      }
   }

   private void useDefaultGains()
   {
      setJointGains(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.0, 0.0, 0.0);
      setJointGains(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0, 0.0, 0.0);
      setJointGains(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), 0.0, 0.0, 0.0);

      setJointGains(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 30.0, 0.0, 3.0);
      setJointGains(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 180.0, 0.0, 12.0);
      setJointGains(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 180.0, 0.0, 12.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 15.0, 0.0, 1.5);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), 15.0, 0.0, 1.5);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 12.0, 0.0, 1.2);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), 12.0, 0.0, 3.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), 15.0, 0.0, 0.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 6.0, 0.0, 0.3);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 6.0, 0.0, 0.3);

         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 30.0, 0.0, 6.2);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 150.0, 0.0, 7.5);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), 160.0, 0.0, 7.0);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 75.0, 0.0, 3.0);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), 25.0, 0.0, 2.0);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 17.0, 0.0, 1.0);
      }
   }

   private void setJointGains(String jointName, double kp, double ki, double kd)
   {
      setProportionalGain(jointName, kp);
      setIntegralGain(jointName, ki);
      setDerivativeGain(jointName, kd);
   }

   private void setProportionalGain(String jointName, double value)
   {
      proportionalGain.put(jointName, value);
   }

   private void setDerivativeGain(String jointName, double value)
   {
      derivativeGain.put(jointName, value);
   }

   private void setIntegralGain(String jointName, double value)
   {
      integralGain.put(jointName, value);
   }

   @Override
   public double getProportionalGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getProportionalGain(jointName);
   }

   @Override
   public double getProportionalGain(String jointName)
   {
      if (proportionalGain.containsKey(jointName))
         return proportionalGain.get(jointName);
      else
         return 0.0;
   }

   @Override
   public double getDerivativeGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getDerivativeGain(jointName);
   }

   @Override
   public double getDerivativeGain(String jointName)
   {
      if (derivativeGain.containsKey(jointName))
         return derivativeGain.get(jointName);
      else
         return 0.0;
   }

   @Override
   public double getIntegralGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getIntegralGain(jointName);
   }

   @Override
   public double getIntegralGain(String jointName)
   {
      if (integralGain.containsKey(jointName))
         return integralGain.get(jointName);
      else
         return 0.0;
   }

   @Override
   public double getPositionControlMasterGain()
   {
      return 1.0;
   }

   private void setSetpoint(String jointName, double value)
   {
      setPoints.put(jointName, value);
   }

   @Override
   public double getSetpoint(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getSetpoint(jointName);
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
