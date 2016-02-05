package us.ihmc.aware.vmc;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;

public class QuadrupedJointLimits
{
   private final static double DEFAULT_POSITION_LOWER_LIMIT = -2 * Math.PI;
   private final static double DEFAULT_POSITION_UPPER_LIMIT = 2 * Math.PI;
   private final static double DEFAULT_SOFT_POSITION_LOWER_LIMIT = -2 * Math.PI;
   private final static double DEFAULT_SOFT_POSITION_UPPER_LIMIT = 2 * Math.PI;
   private final static double DEFAULT_EFFORT_LIMIT = 1000.0;
   private final static double DEFAULT_VELOCITY_LIMIT = 100.0;
   private final static double DEFAULT_ACCELERATION_LIMIT = 10000.0;
   private final TObjectDoubleHashMap<QuadrupedJointName> positionLowerLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> positionUpperLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> softPositionLowerLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> softPositionUpperLimit = new TObjectDoubleHashMap<>();
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
      return positionLowerLimit.contains(jointName) ? positionLowerLimit.get(jointName) : DEFAULT_POSITION_LOWER_LIMIT;
   }

   public double getPositionUpperLimit(QuadrupedJointName jointName)
   {
      return positionUpperLimit.contains(jointName) ? positionUpperLimit.get(jointName) : DEFAULT_POSITION_UPPER_LIMIT;
   }

   public double getSoftPositionLowerLimit(QuadrupedJointName jointName)
   {
      return softPositionLowerLimit.contains(jointName) ? softPositionLowerLimit.get(jointName) : DEFAULT_SOFT_POSITION_LOWER_LIMIT;
   }

   public double getSoftPositionUpperLimit(QuadrupedJointName jointName)
   {
      return softPositionUpperLimit.contains(jointName) ? softPositionUpperLimit.get(jointName) : DEFAULT_SOFT_POSITION_UPPER_LIMIT;
   }

   public double getEffortLimit(QuadrupedJointName jointName)
   {
      return effortLimit.contains(jointName) ? effortLimit.get(jointName) : DEFAULT_EFFORT_LIMIT;
   }

   public double getVelocityLimit(QuadrupedJointName jointName)
   {
      return velocityLimit.contains(jointName) ? velocityLimit.get(jointName) : DEFAULT_VELOCITY_LIMIT;
   }

   public double getAccelerationLimit(QuadrupedJointName jointName)
   {
      return accelerationLimit.contains(jointName) ? accelerationLimit.get(jointName) : DEFAULT_ACCELERATION_LIMIT;
   }
}
