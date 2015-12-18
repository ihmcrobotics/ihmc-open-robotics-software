package us.ihmc.quadrupedRobotics.virtualModelController;

import gnu.trove.map.hash.TObjectDoubleHashMap;

public class QuadrupedJointLimits
{
   private final TObjectDoubleHashMap<String> positionLowerLimit = new TObjectDoubleHashMap<String>();
   private final TObjectDoubleHashMap<String> positionUpperLimit = new TObjectDoubleHashMap<String>();
   private final TObjectDoubleHashMap<String> softPositionLowerLimit = new TObjectDoubleHashMap<String>();
   private final TObjectDoubleHashMap<String> softPositionUpperLimit = new TObjectDoubleHashMap<String>();
   private final TObjectDoubleHashMap<String> softPositionLimitStiffness = new TObjectDoubleHashMap<String>();
   private final TObjectDoubleHashMap<String> softPositionLimitDamping = new TObjectDoubleHashMap<String>();
   private final TObjectDoubleHashMap<String> effortLimit = new TObjectDoubleHashMap<String>();
   private final TObjectDoubleHashMap<String> velocityLimit = new TObjectDoubleHashMap<String>();
   private final TObjectDoubleHashMap<String> accelerationLimit = new TObjectDoubleHashMap<String>();

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

   public void setPositionLowerLimit(String jointName, double value)
   {
      positionLowerLimit.put(jointName, value);
   }

   public void setPositionUpperLimit(String jointName, double value)
   {
      positionUpperLimit.put(jointName, value);
   }

   public void setSoftPositionLowerLimit(String jointName, double value)
   {
      softPositionLowerLimit.put(jointName, value);
   }

   public void setSoftPositionUpperLimit(String jointName, double value)
   {
      softPositionUpperLimit.put(jointName, value);
   }

   public void setSoftPositionLimitStiffness(String jointName, double value)
   {
      softPositionLimitStiffness.put(jointName, value);
   }

   public void setSoftPositionLimitDamping(String jointName, double value)
   {
      softPositionLimitDamping.put(jointName, value);
   }

   public void setEffortLimit(String jointName, double value)
   {
      effortLimit.put(jointName, value);
   }

   public void setVelocityLimit(String jointName, double value)
   {
      velocityLimit.put(jointName, value);
   }

   public void setAccelerationLimit(String jointName, double value)
   {
      accelerationLimit.put(jointName, value);
   }

   public double getPositionLowerLimit(String jointName)
   {
      return positionLowerLimit.contains(jointName) ? positionLowerLimit.get(jointName) : -2 * Math.PI;
   }

   public double getPositionUpperLimit(String jointName)
   {
      return positionUpperLimit.contains(jointName) ? positionUpperLimit.get(jointName) : 2 * Math.PI;
   }

   public double getSoftPositionLowerLimit(String jointName)
   {
      return softPositionLowerLimit.contains(jointName) ? softPositionLowerLimit.get(jointName) : -2 * Math.PI;
   }

   public double getSoftPositionUpperLimit(String jointName)
   {
      return softPositionUpperLimit.contains(jointName) ? softPositionUpperLimit.get(jointName) : 2 * Math.PI;
   }

   public double getSoftPositionLimitStiffness(String jointName)
   {
      return softPositionLimitStiffness.contains(jointName) ? softPositionLimitStiffness.get(jointName) : 100.0;
   }

   public double getSoftPositionLimitDamping(String jointName)
   {
      return softPositionLimitDamping.contains(jointName) ? softPositionLimitDamping.get(jointName) : 10.0;
   }

   public double getEffortLimit(String jointName)
   {
      return effortLimit.contains(jointName) ? effortLimit.get(jointName) : 1000.0;
   }

   public double getVelocityLimit(String jointName)
   {
      return velocityLimit.contains(jointName) ? velocityLimit.get(jointName) : 100.0;
   }

   public double getAccelerationLimit(String jointName)
   {
      return accelerationLimit.contains(jointName) ? accelerationLimit.get(jointName) : 10000.0;
   }
}
