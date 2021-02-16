package us.ihmc.robotics.screwTheory;

import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoInvertedFourBarJoint extends InvertedFourBarJoint
{
   private final YoDouble q, qd, qdd, tau;
   private final YoDouble jointLimitLower, jointLimitUpper;
   private final YoDouble velocityLimitLower, velocityLimitUpper;
   private final YoDouble effortLimitLower, effortLimitUpper;

   public YoInvertedFourBarJoint(String name, RevoluteJointBasics[] fourBarJoints, int masterJointIndex, YoRegistry registry)
   {
      super(name, fourBarJoints, masterJointIndex);

      q = new YoDouble("q_" + getName(), registry);
      qd = new YoDouble("qd_" + getName(), registry);
      qdd = new YoDouble("qdd_" + getName(), registry);
      tau = new YoDouble("tau_" + getName(), registry);
      jointLimitLower = new YoDouble("q_min_" + getName(), registry);
      jointLimitUpper = new YoDouble("q_max_" + getName(), registry);
      velocityLimitLower = new YoDouble("qd_min_" + getName(), registry);
      velocityLimitUpper = new YoDouble("qd_max_" + getName(), registry);
      effortLimitLower = new YoDouble("tau_min_" + getName(), registry);
      effortLimitUpper = new YoDouble("tau_max_" + getName(), registry);
   }

   /** {@inheritDoc} */
   @Override
   public void setQ(double q)
   {
      super.setQ(q);
      this.q.set(q);
   }

   /** {@inheritDoc} */
   @Override
   public void setQd(double qd)
   {
      super.setQd(qd);
      this.qd.set(qd);
   }

   /** {@inheritDoc} */
   @Override
   public void setQdd(double qdd)
   {
      super.setQdd(qdd);
      this.qdd.set(qdd);
   }

   /** {@inheritDoc} */
   @Override
   public void setTau(double tau)
   {
      super.setTau(tau);
      this.tau.set(tau);
   }

   /** {@inheritDoc} */
   @Override
   public void setJointLimits(double jointLimitLower, double jointLimitUpper)
   {
      super.setJointLimits(jointLimitLower, jointLimitUpper);
      this.jointLimitLower.set(jointLimitLower);
      this.jointLimitUpper.set(jointLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   public void setVelocityLimits(double velocityLimitLower, double velocityLimitUpper)
   {
      super.setVelocityLimits(velocityLimitLower, velocityLimitUpper);
      this.velocityLimitLower.set(velocityLimitLower);
      this.velocityLimitUpper.set(velocityLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   public void setEffortLimits(double effortLimitLower, double effortLimitUpper)
   {
      super.setEffortLimits(effortLimitLower, effortLimitUpper);
      this.effortLimitLower.set(effortLimitLower);
      this.effortLimitUpper.set(effortLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   public double getQ()
   {
      q.set(super.getQ());
      return q.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getQd()
   {
      qd.set(super.getQd());
      return qd.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getQdd()
   {
      qdd.set(super.getQdd());
      return qdd.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getTau()
   {
      tau.set(super.getTau());
      return tau.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getJointLimitLower()
   {
      jointLimitLower.set(super.getJointLimitLower());
      return jointLimitLower.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getJointLimitUpper()
   {
      jointLimitUpper.set(super.getJointLimitUpper());
      return jointLimitUpper.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitLower()
   {
      velocityLimitLower.set(super.getVelocityLimitLower());
      return velocityLimitLower.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitUpper()
   {
      velocityLimitUpper.set(super.getVelocityLimitUpper());
      return velocityLimitUpper.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitLower()
   {
      effortLimitLower.set(super.getEffortLimitLower());
      return effortLimitLower.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitUpper()
   {
      effortLimitUpper.set(super.getEffortLimitUpper());
      return effortLimitUpper.getValue();
   }
}
