package us.ihmc.aware.vmc;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;

public class QuadrupedJointLimits
{
   private double defaultPositionLowerLimit;
   private double defaultPositionUpperLimit;
   private double defaultSoftPositionLowerLimit;
   private double defaultSoftPositionUpperLimit;
   private double defaultEffortLimit;
   private double defaultVelocityLimit;
   private double defaultAccelerationLimit;
   private final TObjectDoubleHashMap<QuadrupedJointName> positionLowerLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> positionUpperLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> softPositionLowerLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> softPositionUpperLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> effortLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> velocityLimit = new TObjectDoubleHashMap<>();
   private final TObjectDoubleHashMap<QuadrupedJointName> accelerationLimit = new TObjectDoubleHashMap<>();

   public QuadrupedJointLimits()
   {
      setDefaults();
   }

   public QuadrupedJointLimits(QuadrupedJointLimits jointLimits)
   {
      this.defaultPositionLowerLimit = jointLimits.defaultPositionLowerLimit;
      this.defaultPositionUpperLimit = jointLimits.defaultPositionUpperLimit;
      this.defaultSoftPositionLowerLimit = jointLimits.defaultSoftPositionLowerLimit;
      this.defaultSoftPositionUpperLimit = jointLimits.defaultSoftPositionUpperLimit;
      this.defaultEffortLimit = jointLimits.defaultEffortLimit;
      this.defaultVelocityLimit = jointLimits.defaultVelocityLimit;
      this.defaultAccelerationLimit = jointLimits.defaultAccelerationLimit;

      positionLowerLimit.putAll(jointLimits.positionLowerLimit);
      positionUpperLimit.putAll(jointLimits.positionUpperLimit);
      softPositionLowerLimit.putAll(jointLimits.softPositionLowerLimit);
      softPositionUpperLimit.putAll(jointLimits.softPositionUpperLimit);
      effortLimit.putAll(jointLimits.effortLimit);
      velocityLimit.putAll(jointLimits.velocityLimit);
      accelerationLimit.putAll(jointLimits.accelerationLimit);
   }

   public void setDefaults()
   {
      defaultPositionLowerLimit = -2 * Math.PI;
      defaultPositionUpperLimit = 2 * Math.PI;
      defaultSoftPositionLowerLimit = -2 * Math.PI;
      defaultSoftPositionUpperLimit = 2 * Math.PI;
      defaultEffortLimit = 1000.0;
      defaultVelocityLimit = 100.0;
      defaultAccelerationLimit = 10000.0;

      positionLowerLimit.clear();
      positionUpperLimit.clear();
      softPositionLowerLimit.clear();
      softPositionUpperLimit.clear();
      effortLimit.clear();
      velocityLimit.clear();
      accelerationLimit.clear();
   }

   public void setPositionLowerLimit(double value)
   {
      defaultPositionLowerLimit = value;
      positionLowerLimit.clear();
   }

   public void setPositionUpperLimit(double value)
   {
      defaultPositionUpperLimit = value;
      positionUpperLimit.clear();
   }

   public void setSoftPositionLowerLimit(double value)
   {
      defaultSoftPositionLowerLimit = value;
      softPositionLowerLimit.clear();
   }

   public void setSoftPositionUpperLimit(double value)
   {
      defaultSoftPositionUpperLimit = value;
      softPositionUpperLimit.clear();
   }

   public void setEffortLimit(double value)
   {
      defaultEffortLimit = value;
      effortLimit.clear();
   }

   public void setVelocityLimit(double value)
   {
      defaultVelocityLimit = value;
      velocityLimit.clear();
   }

   public void setAccelerationLimit(double value)
   {
      defaultAccelerationLimit = value;
      accelerationLimit.clear();
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
      return positionLowerLimit.contains(jointName) ? positionLowerLimit.get(jointName) : defaultPositionLowerLimit;
   }

   public double getPositionUpperLimit(QuadrupedJointName jointName)
   {
      return positionUpperLimit.contains(jointName) ? positionUpperLimit.get(jointName) : defaultPositionUpperLimit;
   }

   public double getSoftPositionLowerLimit(QuadrupedJointName jointName)
   {
      return softPositionLowerLimit.contains(jointName) ? softPositionLowerLimit.get(jointName) : defaultSoftPositionLowerLimit;
   }

   public double getSoftPositionUpperLimit(QuadrupedJointName jointName)
   {
      return softPositionUpperLimit.contains(jointName) ? softPositionUpperLimit.get(jointName) : defaultSoftPositionUpperLimit;
   }

   public double getEffortLimit(QuadrupedJointName jointName)
   {
      return effortLimit.contains(jointName) ? effortLimit.get(jointName) : defaultEffortLimit;
   }

   public double getVelocityLimit(QuadrupedJointName jointName)
   {
      return velocityLimit.contains(jointName) ? velocityLimit.get(jointName) : defaultVelocityLimit;
   }

   public double getAccelerationLimit(QuadrupedJointName jointName)
   {
      return accelerationLimit.contains(jointName) ? accelerationLimit.get(jointName) : defaultAccelerationLimit;
   }
}
