package us.ihmc.valkyrie;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.PositionControlParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;

public class ValkyrieWalkingPositionControlParameters implements PositionControlParameters
{
   private final Map<String, Double> positionGains = new HashMap<>();
   private final Map<String, Double> derivativeGains = new HashMap<>();
   private final ValkyrieJointMap jointMap;

   public ValkyrieWalkingPositionControlParameters(RobotTarget robotTarget, ValkyrieJointMap jointMap)
   {
      this.jointMap = jointMap;

      boolean runningOnRealRobot = robotTarget == RobotTarget.REAL_ROBOT;

      if (runningOnRealRobot)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            // Can go up to kp = 30.0, kd = 3.0
            setGains(robotSide, LegJointName.HIP_YAW, 15.0, 1.5);
            setGains(robotSide, LegJointName.HIP_ROLL, 15.0, 1.5);
            setGains(robotSide, LegJointName.HIP_PITCH, 15.0, 1.5);
            // Can go up to kp = 30.0, kd = 4.0
            setGains(robotSide, LegJointName.KNEE_PITCH, 15.0, 2.0);
            // Can go up to kp = 60.0, kd = 6.0
            setGains(robotSide, LegJointName.ANKLE_PITCH, 30.0, 3.0);
            setGains(robotSide, LegJointName.ANKLE_ROLL, 30.0, 3.0);
            
            // Can go up to kp = 30.0, kd = 2.0
            setGains(robotSide, ArmJointName.SHOULDER_PITCH, 15.0, 1.0);
            setGains(robotSide, ArmJointName.SHOULDER_ROLL, 15.0, 1.0);
            setGains(robotSide, ArmJointName.SHOULDER_YAW, 15.0, 1.0);
            // Can go up to kp = 30.0, kd = 2.0
            setGains(robotSide, ArmJointName.ELBOW_PITCH, 15.0, 1.0);
            
            setGains(robotSide, ArmJointName.ELBOW_ROLL, 7.0, 0.0);
            setGains(robotSide, ArmJointName.WRIST_ROLL, 20.0, 0.5);
            setGains(robotSide, ArmJointName.FIRST_WRIST_PITCH, 20.0, 0.5);
         }
         
         // Can go up to kp = 50.0, kd = 1.0
         setGains(SpineJointName.SPINE_YAW, 30.0, 1.0);
         setGains(SpineJointName.SPINE_PITCH, 30.0, 1.0);
         setGains(SpineJointName.SPINE_ROLL, 30.0, 1.0);
      }
   }

   private void setGains(SpineJointName spineJointName, double kp, double kd)
   {
      String jointName = jointMap.getSpineJointName(spineJointName);
      setGains(jointName, kp, kd);
   }

   private void setGains(RobotSide robotSide, LegJointName legJointName, double kp, double kd)
   {
      String jointName = jointMap.getLegJointName(robotSide, legJointName);
      setGains(jointName, kp, kd);
   }

   private void setGains(RobotSide robotSide, ArmJointName armJointName, double kp, double kd)
   {
      String jointName = jointMap.getArmJointName(robotSide, armJointName);
      setGains(jointName, kp, kd);
   }

   private void setGains(String jointName, double kp, double kd)
   {
      positionGains.put(jointName, kp);
      derivativeGains.put(jointName, kd);
   }

   @Override
   public double getProportionalGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getProportionalGain(jointName);
   }

   @Override
   public double getDerivativeGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getDerivativeGain(jointName);
   }

   @Override
   public double getIntegralGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getIntegralGain(jointName);
   }

   @Override
   public double getProportionalGain(String jointName)
   {
      if (positionGains.containsKey(jointName))
         return positionGains.get(jointName);
      else
         return 0;
   }

   @Override
   public double getDerivativeGain(String jointName)
   {
      if (derivativeGains.containsKey(jointName))
         return derivativeGains.get(jointName);
      else
         return 0.0;
   }

   @Override
   public double getIntegralGain(String jointName)
   {
      return 0;
   }

   @Override
   public double getPositionControlMasterGain()
   {
      return 1.0;
   }

}
