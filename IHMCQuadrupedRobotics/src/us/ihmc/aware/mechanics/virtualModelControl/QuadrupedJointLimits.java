package us.ihmc.aware.mechanics.virtualModelControl;

import java.util.EnumMap;
import java.util.Map;

import com.google.common.base.CaseFormat;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
import us.ihmc.aware.params.DoubleArrayParameter;
import us.ihmc.aware.params.DoubleParameter;
import us.ihmc.aware.params.ParameterFactory;

public class QuadrupedJointLimits
{
   private final ParameterFactory parameterFactory = new ParameterFactory(QuadrupedJointLimits.class);
   private final Map<QuadrupedJointName, DoubleArrayParameter> positionLimits = new EnumMap<>(QuadrupedJointName.class);
   private final Map<QuadrupedJointName, DoubleArrayParameter> softPositionLimits = new EnumMap<>(QuadrupedJointName.class);
   private final Map<QuadrupedJointName, DoubleParameter> effortLimit = new EnumMap<>(QuadrupedJointName.class);
   private final Map<QuadrupedJointName, DoubleParameter> velocityLimit = new EnumMap<>(QuadrupedJointName.class);
   private final Map<QuadrupedJointName, DoubleParameter> accelerationLimit = new EnumMap<>(QuadrupedJointName.class);

   public QuadrupedJointLimits()
   {
      for (QuadrupedJointName jointName : QuadrupedJointName.values())
      {
         String upperJointName = CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, jointName.toString());
         positionLimits.put(jointName, parameterFactory.createDoubleArray("positionLimits" + upperJointName, -2 * Math.PI, 2 * Math.PI));
         softPositionLimits.put(jointName, parameterFactory.createDoubleArray("softPositionLimits" + upperJointName, -2 * Math.PI, 2 * Math.PI));
         effortLimit.put(jointName, parameterFactory.createDouble("effortLimit" + upperJointName, 1000.0));
         velocityLimit.put(jointName, parameterFactory.createDouble("velocityLimit" + upperJointName, 100.0));
         accelerationLimit.put(jointName, parameterFactory.createDouble("accelerationLimit" + upperJointName, 10000.0));
      }
   }

   public double getPositionLowerLimit(QuadrupedJointName jointName)
   {
      return positionLimits.get(jointName).get(0);
   }

   public double getPositionUpperLimit(QuadrupedJointName jointName)
   {
      return positionLimits.get(jointName).get(1);
   }

   public double getSoftPositionLowerLimit(QuadrupedJointName jointName)
   {
      return softPositionLimits.get(jointName).get(0);
   }

   public double getSoftPositionUpperLimit(QuadrupedJointName jointName)
   {
      return softPositionLimits.get(jointName).get(1);
   }

   public double getEffortLimit(QuadrupedJointName jointName)
   {
      return effortLimit.get(jointName).get();
   }

   public double getVelocityLimit(QuadrupedJointName jointName)
   {
      return velocityLimit.get(jointName).get();
   }

   public double getAccelerationLimit(QuadrupedJointName jointName)
   {
      return accelerationLimit.get(jointName).get();
   }
}
