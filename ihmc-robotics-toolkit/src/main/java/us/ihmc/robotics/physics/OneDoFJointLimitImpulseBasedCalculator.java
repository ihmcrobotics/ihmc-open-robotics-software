package us.ihmc.robotics.physics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.MultiBodyResponseCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class OneDoFJointLimitImpulseBasedCalculator implements ImpulseBasedConstraintCalculator
{
   private double springConstant = 5.0;

   private final double dt;
   private final RigidBodyBasics rootBody;
   private final OneDoFJointBasics joint;
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;
   private final MultiBodyResponseCalculator responseCalculator;

   private boolean isInitialized = false;
   private boolean isInertiaUpToDate = false;
   private boolean isConstraintActive = false;

   private JointStateProvider externalJointTwistModifier;

   private double jointVelocityNoImpulse = 0.0;
   private double jointVelocityDueToOtherImpulse = 0.0;
   private double jointVelocity = 0.0;
   private double previousJointVelocity = 0.0;
   private double jointVelocityUpdate = 0.0;

   private double inverseApparentInertia = 0.0;
   private double impulse = 0.0;
   private double previousImpulse = 0.0;
   private double impulseUpdate = 0.0;

   public OneDoFJointLimitImpulseBasedCalculator(double dt, OneDoFJointBasics joint, ForwardDynamicsCalculator forwardDynamicsCalculator)
   {
      this.dt = dt;
      this.joint = joint;
      this.forwardDynamicsCalculator = forwardDynamicsCalculator;

      rootBody = MultiBodySystemTools.getRootBody(joint.getPredecessor());
      responseCalculator = new MultiBodyResponseCalculator(forwardDynamicsCalculator);
   }

   @Override
   public void reset()
   {
      isInitialized = false;
   }

   @Override
   public void initialize()
   {
      if (isInitialized)
         return;

      jointVelocityNoImpulse = joint.getQ() + dt * forwardDynamicsCalculator.getComputedJointAcceleration(joint).get(0);

      isInertiaUpToDate = false;
      isInitialized = true;
   }

   @Override
   public void updateImpulse(double alpha)
   {
      boolean isFirstUpdate = !isInitialized;
      initialize();

      if (externalJointTwistModifier != null)
      {
         jointVelocityDueToOtherImpulse = externalJointTwistModifier.getJointState(joint);
         jointVelocity = jointVelocityNoImpulse + jointVelocityDueToOtherImpulse;
      }
      else
      {
         jointVelocityDueToOtherImpulse = 0.0;
         jointVelocity = jointVelocityNoImpulse;
      }

      if (isFirstUpdate)
      {
         previousJointVelocity = jointVelocity;
         jointVelocityUpdate = jointVelocity;
      }
      else
      {
         jointVelocityUpdate = jointVelocity - previousJointVelocity;
         previousJointVelocity = jointVelocity;
      }

      double distanceToLowerLimit = joint.getQ() - joint.getJointLimitLower();
      double distanceToUpperLimit = joint.getQ() - joint.getJointLimitUpper();

      if (distanceToLowerLimit <= 0.0)
      { // Violating lower limit
         jointVelocity += springConstant * distanceToLowerLimit;
         isConstraintActive = jointVelocity <= 0.0;
      }
      else if (distanceToUpperLimit >= 0.0)
      { // Violating upper limit
         jointVelocity += springConstant * distanceToUpperLimit;
         isConstraintActive = jointVelocity >= 0.0;
      }
      else
      {
         isConstraintActive = false;
      }

      if (isConstraintActive)
      {
         if (!isInertiaUpToDate)
         {
            inverseApparentInertia = responseCalculator.computeJointApparentInertiaInverse(joint);
            isInertiaUpToDate = true;
         }

         impulse = -inverseApparentInertia * jointVelocity;
         responseCalculator.applyJointImpulse(joint, impulse);
      }
      else
      {
         responseCalculator.reset();
      }

      if (isFirstUpdate)
      {
         impulseUpdate = impulse;
      }
      else
      {
         impulse = EuclidCoreTools.interpolate(previousImpulse, impulse, alpha);
         impulseUpdate = impulse - previousImpulse;
      }

      previousImpulse = impulse;
   }

   @Override
   public double getImpulseUpdate()
   {
      return impulseUpdate;
   }

   @Override
   public double getVelocityUpdate()
   {
      return jointVelocityUpdate;
   }

   @Override
   public boolean isConstraintActive()
   {
      return isConstraintActive;
   }

   @Override
   public double getDT()
   {
      return dt;
   }

   public OneDoFJointBasics getJoint()
   {
      return joint;
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculator()
   {
      return forwardDynamicsCalculator;
   }

   public DenseMatrix64F computeJointVelocityChange()
   {
      if (!isConstraintActive)
         return null;

      return responseCalculator.propagateImpulse();
   }

   @Override
   public void setExternalTwistModifier(JointStateProvider externalJointTwistModifier)
   {
      this.externalJointTwistModifier = externalJointTwistModifier;
   }

   @Override
   public int getNumberOfRobotsInvolved()
   {
      return 1;
   }

   @Override
   public RigidBodyBasics getRootBody(int index)
   {
      return rootBody;
   }

   @Override
   public RigidBodyTwistProvider getRigidBodyTwistChangeProvider(int index)
   {
      return responseCalculator.getTwistChangeProvider();
   }

   @Override
   public JointStateProvider getJointTwistChangeProvider(int index)
   {
      return JointStateProvider.toJointTwistProvider(responseCalculator);
   }

   @Override
   public DenseMatrix64F getJointVelocityChange(int index)
   {
      if (!isConstraintActive)
         return null;
      else
         return responseCalculator.propagateImpulse();
   }
}
