package us.ihmc.valkyrie;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.PositionControlParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;

public class ValkyrieWalkingPositionControlParameters implements PositionControlParameters
{
   private final Map<String, Double> derivativeGains = new HashMap<>();

   public ValkyrieWalkingPositionControlParameters(ValkyrieJointMap jointMap)
   {
      double legKd = 4.75;
      double armKd = 6.00;

      for (RobotSide robotSide : RobotSide.values)
      {
         derivativeGains.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), legKd);
         derivativeGains.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), legKd);
         derivativeGains.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), legKd);
         derivativeGains.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), legKd);
         derivativeGains.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), legKd);
         derivativeGains.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), legKd);

         derivativeGains.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), armKd);
         derivativeGains.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), armKd);
         derivativeGains.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), armKd);
         derivativeGains.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), armKd);
      }
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
