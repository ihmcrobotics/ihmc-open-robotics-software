package us.ihmc.aware.vmc;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;

public class QuadrupedVirtualModelControllerSettings
{
   private double defaultJointDamping = 0.0;
   private double defaultJointPositionLimitStiffness = 200.0;
   private double defaultJointPositionLimitDamping = 5.0;
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

   public void setJointDamping(double value)
   {
      defaultJointDamping = value;
      jointDamping.clear();
   }

   public void setJointDamping(QuadrupedJointName jointName, double value)
   {
      jointDamping.put(jointName, value);
   }

   public void setJointPositionLimitStiffness(double value)
   {
      defaultJointPositionLimitStiffness = value;
      jointPositionLimitStiffness.clear();
   }

   public void setJointPositionLimitStiffness(QuadrupedJointName jointName, double value)
   {
      jointPositionLimitStiffness.put(jointName, value);
   }

   public void setJointPositionLimitDamping(double value)
   {
      defaultJointPositionLimitDamping = value;
      jointPositionLimitDamping.clear();
   }
   public void setJointPositionLimitDamping(QuadrupedJointName jointName, double value)
   {
      jointPositionLimitDamping.put(jointName, value);
   }

   public double getJointDamping(QuadrupedJointName jointName)
   {
      return jointDamping.contains(jointName) ? jointDamping.get(jointName) : defaultJointDamping;
   }

   public double getJointPositionLimitStiffness(QuadrupedJointName jointName)
   {
      return jointPositionLimitStiffness.contains(jointName) ? jointPositionLimitStiffness.get(jointName) : defaultJointPositionLimitStiffness;
   }

   public double getJointPositionLimitDamping(QuadrupedJointName jointName)
   {
      return jointPositionLimitDamping.contains(jointName) ? jointPositionLimitDamping.get(jointName) : defaultJointPositionLimitDamping;
   }
}
