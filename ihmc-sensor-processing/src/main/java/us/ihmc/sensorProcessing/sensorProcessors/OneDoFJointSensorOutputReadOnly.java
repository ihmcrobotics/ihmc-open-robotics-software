package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public interface OneDoFJointSensorOutputReadOnly
{
   String getJointName();

   double getPosition();

   double getVelocity();

   double getAcceleration();

   double getEffort();

   boolean isJointEnabled();

   public static OneDoFJointSensorOutputReadOnly createFromOneDoFJoint(OneDoFJointReadOnly joint, boolean isEnabled)
   {
      return createFromOneDoFJoint(joint, () -> true);
   }

   public static OneDoFJointSensorOutputReadOnly createFromOneDoFJoint(OneDoFJointReadOnly joint, BooleanSupplier isEnabled)
   {
      return createFromSuppliers(joint.getName(), joint::getQ, joint::getQd, joint::getQdd, joint::getTau, isEnabled);
   }

   public static OneDoFJointSensorOutputReadOnly createFromProviders(String jointName, DoubleProvider position, DoubleProvider velocity,
                                                                     DoubleProvider acceleration, DoubleProvider effort, BooleanProvider isEnabled)
   {
      return createFromSuppliers(jointName, position::getValue, velocity::getValue, acceleration::getValue, effort::getValue, isEnabled::getValue);
   }

   public static OneDoFJointSensorOutputReadOnly createFromSuppliers(String jointName, DoubleSupplier position, DoubleSupplier velocity,
                                                                     DoubleSupplier acceleration, DoubleSupplier effort, BooleanSupplier isEnabled)
   {
      return new OneDoFJointSensorOutputReadOnly()
      {
         @Override
         public String getJointName()
         {
            return jointName;
         }

         @Override
         public double getPosition()
         {
            return position.getAsDouble();
         }

         @Override
         public double getVelocity()
         {
            return velocity.getAsDouble();
         }

         @Override
         public double getAcceleration()
         {
            return acceleration.getAsDouble();
         }

         @Override
         public double getEffort()
         {
            return effort.getAsDouble();
         }

         @Override
         public boolean isJointEnabled()
         {
            return isEnabled.getAsBoolean();
         }
      };
   }
}
