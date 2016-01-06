package us.ihmc.quadrupedRobotics.virtualModelController;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;

public class QuadrupedJointLimits
{
   private final TObjectDoubleHashMap<QuadrupedJointName> positionLowerLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> positionUpperLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> softPositionLowerLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> softPositionUpperLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> softPositionLimitStiffness = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> softPositionLimitDamping = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> effortLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> velocityLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> accelerationLimit = new TObjectDoubleHashMap<>();

   public QuadrupedJointLimits()
   {
   }

   public QuadrupedJointLimits(QuadrupedJointLimits jointLimits)
   {
      positionLowerLimit.putAll(jointLimits.positionLowerLimit);
      positionUpperLimit.putAll(jointLimits.positionUpperLimit);
      softPositionLowerLimit.putAll(jointLimits.softPositionLowerLimit);
      softPositionUpperLimit.putAll(jointLimits.softPositionUpperLimit);
      softPositionLimitStiffness.putAll(jointLimits.softPositionLimitStiffness);
      softPositionLimitDamping.putAll(jointLimits.softPositionLimitDamping);
      effortLimit.putAll(jointLimits.effortLimit);
      velocityLimit.putAll(jointLimits.velocityLimit);
      accelerationLimit.putAll(jointLimits.accelerationLimit);
   }

   public void setPositionLowerLimit(QuadrupedJointName jointName, double value)
   {
      positionLowerLimit.put(jointName, value);
   }

   public void setPositionUpperLimit(QuadrupedJointName jointName, double value)
   {
      positionUpperLimit.put(jointName, value);
   }

   public void setSoftPositionLowerLimit(QuadrupedJointName jointName, double value)
   {
      softPositionLowerLimit.put(jointName, value);
   }

   public void setSoftPositionUpperLimit(QuadrupedJointName jointName, double value)
   {
      softPositionUpperLimit.put(jointName, value);
   }

   public void setSoftPositionLimitStiffness(QuadrupedJointName jointName, double value)
   {
      softPositionLimitStiffness.put(jointName, value);
   }

   public void setSoftPositionLimitDamping(QuadrupedJointName jointName, double value)
   {
      softPositionLimitDamping.put(jointName, value);
   }

   public void setEffortLimit(QuadrupedJointName jointName, double value)
   {
      effortLimit.put(jointName, value);
   }

   public void setVelocityLimit(QuadrupedJointName jointName, double value)
   {
      velocityLimit.put(jointName, value);
   }

   public void setAccelerationLimit(QuadrupedJointName jointName, double value)
   {
      accelerationLimit.put(jointName, value);
   }

   public double getPositionLowerLimit(QuadrupedJointName jointName)
   {
      return positionLowerLimit.contains(jointName) ? positionLowerLimit.get(jointName) : -2 * Math.PI;
   }

   public double getPositionUpperLimit(QuadrupedJointName jointName)
   {
      return positionUpperLimit.contains(jointName) ? positionUpperLimit.get(jointName) : 2 * Math.PI;
   }

   public double getSoftPositionLowerLimit(QuadrupedJointName jointName)
   {
      return softPositionLowerLimit.contains(jointName) ? softPositionLowerLimit.get(jointName) : -2 * Math.PI;
   }

   public double getSoftPositionUpperLimit(QuadrupedJointName jointName)
   {
      return softPositionUpperLimit.contains(jointName) ? softPositionUpperLimit.get(jointName) : 2 * Math.PI;
   }

   public double getSoftPositionLimitStiffness(QuadrupedJointName jointName)
   {
      return softPositionLimitStiffness.contains(jointName) ? softPositionLimitStiffness.get(jointName) : 100.0;
   }

   public double getSoftPositionLimitDamping(QuadrupedJointName jointName)
   {
      return softPositionLimitDamping.contains(jointName) ? softPositionLimitDamping.get(jointName) : 10.0;
   }

   public double getEffortLimit(QuadrupedJointName jointName)
   {
      return effortLimit.contains(jointName) ? effortLimit.get(jointName) : 1000.0;
   }

   public double getVelocityLimit(QuadrupedJointName jointName)
   {
      return velocityLimit.contains(jointName) ? velocityLimit.get(jointName) : 100.0;
   }

   public double getAccelerationLimit(QuadrupedJointName jointName)
   {
      return accelerationLimit.contains(jointName) ? accelerationLimit.get(jointName) : 10000.0;
   }
}
