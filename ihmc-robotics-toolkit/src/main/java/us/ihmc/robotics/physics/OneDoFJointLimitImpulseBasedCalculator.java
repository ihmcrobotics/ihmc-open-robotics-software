package us.ihmc.robotics.physics;

import java.util.List;
import java.util.stream.Collectors;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.MultiBodyResponseCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class OneDoFJointLimitImpulseBasedCalculator implements ImpulseBasedConstraintCalculator
{
   public static List<OneDoFJointBasics> findOneDoFJointsAtLimit(RigidBodyBasics rootBody, double dt, ForwardDynamicsCalculator forwardDynamicsCalculator)
   {
      return SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).filter(joint -> isOneDoFJointAtLimit(joint, dt, forwardDynamicsCalculator))
                           .collect(Collectors.toList());
   }

   public static boolean isOneDoFJointAtLimit(OneDoFJointReadOnly joint, double dt, ForwardDynamicsCalculator forwardDynamicsCalculator)
   {
      double q = joint.getQ();
      double qd = joint.getQd();
      double qdd = forwardDynamicsCalculator.getComputedJointAcceleration(joint).get(0);
      double projected_q = q + dt * qd + 0.5 * dt * dt * qdd;
      return projected_q <= joint.getJointLimitLower() || joint.getJointLimitUpper() <= projected_q;
   }

   private double springConstant = 5.0;

   private final double dt;
   private final RigidBodyBasics rootBody;
   private final OneDoFJointBasics joint;
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;
   private final MultiBodyResponseCalculator responseCalculator;

   private boolean isInitialized = false;
   private boolean isInertiaUpToDate = false;
   private boolean isImpulseZero = false;

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

      jointVelocityNoImpulse = joint.getQd() + dt * forwardDynamicsCalculator.getComputedJointAcceleration(joint).get(0);

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

      boolean isNearingLimit;

      if (distanceToLowerLimit <= 0.0)
      { // Violating lower limit
         jointVelocity += springConstant * distanceToLowerLimit;
         isNearingLimit = jointVelocity <= 0.0;
      }
      else if (distanceToUpperLimit >= 0.0)
      { // Violating upper limit
         jointVelocity += springConstant * distanceToUpperLimit;
         isNearingLimit = jointVelocity >= 0.0;
      }
      else
      {
         isNearingLimit = false;
      }

      if (isNearingLimit)
      {
         if (!isInertiaUpToDate)
         {
            inverseApparentInertia = responseCalculator.computeJointApparentInertiaInverse(joint);
            isInertiaUpToDate = true;
         }

         impulse = -jointVelocity / inverseApparentInertia;
         if (Double.isNaN(impulse))
            throw new IllegalStateException("The impulse is NaN");
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

      isImpulseZero = EuclidCoreTools.epsilonEquals(0.0, impulse, 1.0 - 12);

      if (isImpulseZero)
      {
         responseCalculator.reset();
      }
      else
      {
         responseCalculator.applyJointImpulse(joint, impulse);
      }

      previousImpulse = impulse;
   }

   public void setSpringConstant(double springConstant)
   {
      this.springConstant = springConstant;
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
      return !isImpulseZero;
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
      return isImpulseZero ? null : responseCalculator.propagateImpulse();
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
      return isImpulseZero ? null : responseCalculator.propagateImpulse();
   }
}
