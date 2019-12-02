package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public interface OneDoFJointStateReadOnly
{
   /**
    * Gets the name of the joint which this state is for.
    * 
    * @return the joint name.
    */
   String getJointName();

   /**
    * Gets the current joint position.
    * 
    * @return the joint position.
    */
   double getPosition();

   /**
    * Gets the current joint velocity.
    * 
    * @return the joint velocity.
    */
   double getVelocity();

   /**
    * Gets the current joint acceleration.
    * 
    * @return the joint acceleration.
    */
   double getAcceleration();

   /**
    * Gets the current joint effort.
    * 
    * @return the joint effort, i.e. torque or force.
    */
   double getEffort();

   /**
    * Gets the current enabled/disabled state of the joint.
    * 
    * @return {@code true} if the joint is actuated/functioning, {@code false} if the joint is
    *         not-actuated, or fixed, or non-functioning.
    */
   boolean isJointEnabled();

   /**
    * Creates a new state that will reflect the internal state of the given {@code joint} while the
    * {@code isEnabled} state is fixed.
    * 
    * @param joint     the joint to reflect the state of.
    * @param isEnabled whether the state should mark the joint as enabled or disabled.
    * @return the new linked joint state.
    */
   public static OneDoFJointStateReadOnly createFromOneDoFJoint(OneDoFJointReadOnly joint, boolean isEnabled)
   {
      return createFromOneDoFJoint(joint, () -> true);
   }

   /**
    * Creates a new state that will reflect the internal state of the given {@code joint} while the
    * enable/disable state is provided by {@code isEnabled}.
    * 
    * @param joint     the joint to reflect the state of.
    * @param isEnabled the function used to determine the enable/disable state of the joint.
    * @return the new linked joint state.
    */
   public static OneDoFJointStateReadOnly createFromOneDoFJoint(OneDoFJointReadOnly joint, BooleanSupplier isEnabled)
   {
      return createFromSuppliers(joint.getName(), joint::getQ, joint::getQd, joint::getQdd, joint::getTau, isEnabled);
   }

   /**
    * Creates a new state linked to the given providers.
    * 
    * @param jointName    the name of the joint.
    * @param position     the provider for the joint position.
    * @param velocity     the provider for the joint velocity.
    * @param acceleration the provider for the joint acceleration.
    * @param effort       the provider for the joint effort, i.e. torque or force.
    * @param isEnabled    the provider to determine whether the joint is enabled or disabled.
    * @return the new linked joint state.
    */
   public static OneDoFJointStateReadOnly createFromProviders(String jointName, DoubleProvider position, DoubleProvider velocity, DoubleProvider acceleration,
                                                              DoubleProvider effort, BooleanProvider isEnabled)
   {
      return createFromSuppliers(jointName, position::getValue, velocity::getValue, acceleration::getValue, effort::getValue, isEnabled::getValue);
   }

   /**
    * Creates a new state linked to the given suppliers.
    * 
    * @param jointName    the name of the joint.
    * @param position     the supplier for the joint position.
    * @param velocity     the supplier for the joint velocity.
    * @param acceleration the supplier for the joint acceleration.
    * @param effort       the supplier for the joint effort, i.e. torque or force.
    * @param isEnabled    the supplier to determine whether the joint is enabled or disabled.
    * @return the new linked joint state.
    */
   public static OneDoFJointStateReadOnly createFromSuppliers(String jointName, DoubleSupplier position, DoubleSupplier velocity, DoubleSupplier acceleration,
                                                              DoubleSupplier effort, BooleanSupplier isEnabled)
   {
      return new OneDoFJointStateReadOnly()
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
