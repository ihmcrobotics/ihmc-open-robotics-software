package us.ihmc.aware.vmc;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;

public class QuadrupedVirtualModelControllerSettings
{
   private final static double DEFAULT_JOINT_DAMPING = 0.0;
   private final static double DEFAULT_JOINT_POSITION_LIMIT_STIFFNESS = 200.0;
   private final static double DEFAULT_JOINT_POSITION_LIMIT_DAMPING = 20.0;
   private final TObjectDoubleHashMap<QuadrupedJointName> jointDamping = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> jointPositionLimitStiffness = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> jointPositionLimitDamping = new TObjectDoubleHashMap<>();

   public QuadrupedVirtualModelControllerSettings()
   {
   }

   public QuadrupedVirtualModelControllerSettings(QuadrupedVirtualModelControllerSettings quadrupedVirtualModelControllerSettings)
   {
      jointDamping.putAll(quadrupedVirtualModelControllerSettings.jointDamping);
      jointPositionLimitStiffness.putAll(quadrupedVirtualModelControllerSettings.jointPositionLimitStiffness);
      jointPositionLimitDamping.putAll(quadrupedVirtualModelControllerSettings.jointPositionLimitDamping);
   }

   public void setJointDamping(QuadrupedJointName jointName, double value)
   {
      jointDamping.put(jointName, value);
   }

   public void setJointPositionLimitStiffness(QuadrupedJointName jointName, double value)
   {
      jointPositionLimitStiffness.put(jointName, value);
   }

   public void setJointPositionLimitDamping(QuadrupedJointName jointName, double value)
   {
      jointPositionLimitDamping.put(jointName, value);
   }

   public double getJointDamping(QuadrupedJointName jointName)
   {
      return jointDamping.contains(jointName) ? jointDamping.get(jointName) : DEFAULT_JOINT_DAMPING;
   }

   public double getJointPositionLimitStiffness(QuadrupedJointName jointName)
   {
      return jointPositionLimitStiffness.contains(jointName) ? jointPositionLimitStiffness.get(jointName) : DEFAULT_JOINT_POSITION_LIMIT_STIFFNESS;
   }

   public double getJointPositionLimitDamping(QuadrupedJointName jointName)
   {
      return jointPositionLimitDamping.contains(jointName) ? jointPositionLimitDamping.get(jointName) : DEFAULT_JOINT_POSITION_LIMIT_DAMPING;
   }
}
