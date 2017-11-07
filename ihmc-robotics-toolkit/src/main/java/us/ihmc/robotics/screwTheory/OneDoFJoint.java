package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.commons.MathTools;

public abstract class OneDoFJoint extends AbstractInverseDynamicsJoint
{
   protected Twist unitJointTwist;
   protected Twist unitSuccessorTwist;
   protected Twist unitPredecessorTwist;

   protected SpatialAccelerationVector unitJointAcceleration;
   protected SpatialAccelerationVector unitSuccessorAcceleration;
   protected SpatialAccelerationVector unitPredecessorAcceleration;

   private double q;
   private double qd;
   private double qdd;
   private double tauMeasured;
   private double qddDesired;
   
   private double tau;

   private double effortLimitLower = Double.NEGATIVE_INFINITY;
   private double effortLimitUpper = Double.POSITIVE_INFINITY;
   private double jointLimitLower = Double.NEGATIVE_INFINITY;
   private double jointLimitUpper = Double.POSITIVE_INFINITY;
   private double velocityLimitLower = Double.NEGATIVE_INFINITY;
   private double velocityLimitUpper = Double.POSITIVE_INFINITY;

   //   private double tauDamping;

   /*
    * VRC HACKS
    */
   @Deprecated
   private boolean integrateDesiredAccelerations = true; // Hack even more...
   @Deprecated
   private boolean resetDesiredAccelerationIntegrator = false; // Hack even more more...

   @Deprecated
   private boolean resetIntegrator = false;

   private double qDesired;
   private double qdDesired;
   @Deprecated
   private double kp;
   @Deprecated
   private double kd;

   @Deprecated
   private boolean isUnderPositionControl = false;

   private boolean enabled = true;
   @Deprecated
   private boolean useFeedBackForceControl = true;

   /**
    * Describes if a joint is online
    */
   private boolean isOnline = true;

   public OneDoFJoint(String name, RigidBody predecessor, RigidBodyTransform transformToParent)
   {
      super(name, predecessor, transformToParent);
   }

   @Override
   public void getJointTwist(Twist twistToPack)
   {
      twistToPack.set(unitJointTwist);
      twistToPack.scale(qd);
   }

   @Override
   public void getSuccessorTwist(Twist twistToPack)
   {
      twistToPack.set(unitSuccessorTwist);
      twistToPack.scale(qd);
   }

   @Override
   public void getPredecessorTwist(Twist twistToPack)
   {
      twistToPack.set(unitPredecessorTwist);
      twistToPack.scale(qd);
   }

   @Override
   public void getJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(unitJointAcceleration);
      accelerationToPack.scale(qdd);
   }

   @Override
   public void getSuccessorAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(unitSuccessorAcceleration);
      accelerationToPack.scale(qdd);
   }

   @Override
   public void getDesiredJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(unitJointAcceleration);
      accelerationToPack.scale(qddDesired);
   }

   @Override
   public void getDesiredSuccessorAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(unitSuccessorAcceleration);
      accelerationToPack.scale(qddDesired);
   }

   @Override
   public void getDesiredPredecessorAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(unitPredecessorAcceleration);
      accelerationToPack.scale(qddDesired);
   }

   @Override
   public void setJointTorque(DenseMatrix64F matrix, int rowStart)
   {
      setTau(matrix.get(rowStart, 0));
   }

   @Override
   public void getTauMatrix(DenseMatrix64F matrix)
   {
      MathTools.checkIntervalContains(matrix.getNumRows(), 1, Integer.MAX_VALUE);
      MathTools.checkIntervalContains(matrix.getNumCols(), 1, Integer.MAX_VALUE);
      matrix.set(0, 0, tau);
   }

   @Override
   public void getVelocityMatrix(DenseMatrix64F matrix, int rowStart)
   {
      MathTools.checkIntervalContains(matrix.getNumRows(), 1, Integer.MAX_VALUE);
      MathTools.checkIntervalContains(matrix.getNumCols(), 1, Integer.MAX_VALUE);
      matrix.set(rowStart, 0, qd);
   }

   @Override
   public void getDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      MathTools.checkIntervalContains(matrix.getNumRows(), 1, Integer.MAX_VALUE);
      MathTools.checkIntervalContains(matrix.getNumCols(), 1, Integer.MAX_VALUE);
      matrix.set(rowStart, 0, qddDesired);
   }

   @Override
   public void setDesiredAccelerationToZero()
   {
      qddDesired = 0.0;
   }

   @Override
   public void updateMotionSubspace()
   {
      // empty
   }

   @Override
   public void setTorqueFromWrench(Wrench jointWrench)
   {
      unitSuccessorTwist.getBodyFrame().checkReferenceFrameMatch(jointWrench.getBodyFrame());
      unitSuccessorTwist.getExpressedInFrame().checkReferenceFrameMatch(jointWrench.getExpressedInFrame());
      this.tau = unitSuccessorTwist.dot(jointWrench);

      // cheating a little bit; tau = J^T * wrench. J maps joint velocities to joint twists.
      // the unit twist is actually exactly the same as J, except that its entries have different dimensions.
      // we disregard dimensions and just use .dot(.) for efficiency
   }

   @Override
   public int getDegreesOfFreedom()
   {
      return 1;
   }

   @Override
   public void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart)
   {
      setQddDesired(matrix.get(rowStart + 0, 0));
   }

   public double getQ()
   {
      return q;
   }

   public void setQ(double q)
   {
      if (Double.isNaN(q))
         throw new RuntimeException("q is NaN! this = " + this);
      this.q = q;
      getFrameAfterJoint().update();
   }

   public double getQd()
   {
      return qd;
   }

   public void setQd(double qd)
   {
      this.qd = qd;
   }

   public double getQdd()
   {
      return qdd;
   }

   public void setQdd(double qdd)
   {
      this.qdd = qdd;
   }

   
   public double getQddDesired()
   {
      return qddDesired;
   }

  
   public void setQddDesired(double qddDesired)
   {
      this.qddDesired = qddDesired;
   }

   
   public double getTau()
   {
      return tau;
   }

   
   public void setTau(double tau)
   {
      this.tau = tau;
   }

   public void getUnitJointTwist(Twist twistToPack)
   {
      twistToPack.set(unitJointTwist);
   }

   public void getUnitJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(unitJointAcceleration);
   }

   /**
    * Setup the motion subspace for this one DoF joint assuming that the field
    * {@code unitSuccessorTwist} has already been properly setup.
    */
   protected void setMotionSubspace()
   {
      this.motionSubspace = new GeometricJacobian(this, unitSuccessorTwist.getExpressedInFrame());
      this.motionSubspace.compute();
   }

   @Override
   public void getUnitTwist(int dofIndex, Twist unitTwistToPack)
   {
      if (dofIndex != 0)
         throw new ArrayIndexOutOfBoundsException("Illegal index: " + dofIndex + ", was expecting dofIndex equal to 0.");
      unitTwistToPack.set(unitSuccessorTwist);
   }

   @Override
   public void getConfigurationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart, q);
   }

   @Override
   public void setConfiguration(DenseMatrix64F matrix, int rowStart)
   {
      setQ(matrix.get(rowStart, 0));
   }

   @Override
   public void setVelocity(DenseMatrix64F matrix, int rowStart)
   {
      setQd(matrix.get(rowStart, 0));
   }

   @Override
   public int getConfigurationMatrixSize()
   {
      return getDegreesOfFreedom();
   }

   public OneDoFJoint checkAndGetAsOneDoFJoint(InverseDynamicsJoint originalJoint)
   {
      if (originalJoint instanceof OneDoFJoint)
      {
         return (OneDoFJoint) originalJoint;
      }
      else
      {
         throw new RuntimeException("Cannot set " + getClass().getSimpleName() + " to " + originalJoint.getClass().getSimpleName());
      }
   }

   @Override
   public void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint)
   {
      OneDoFJoint oneDoFOriginalJoint = checkAndGetAsOneDoFJoint(originalJoint);
      setQ(oneDoFOriginalJoint.getQ());
      setQd(oneDoFOriginalJoint.getQd());
      setQdd(oneDoFOriginalJoint.getQdd());
      setTauMeasured(oneDoFOriginalJoint.getTauMeasured());
      setEnabled(oneDoFOriginalJoint.isEnabled());
   }

   @Override
   public void setQddDesired(InverseDynamicsJoint originalJoint)
   {
      OneDoFJoint oneDoFOriginalJoint = checkAndGetAsOneDoFJoint(originalJoint);
      setQddDesired(oneDoFOriginalJoint.getQddDesired());
   }

   public double getJointLimitLower()
   {
      return jointLimitLower;
   }

   public void setJointLimitLower(double jointLimitLower)
   {
      this.jointLimitLower = jointLimitLower;
   }
   
   public double getJointLimitUpper()
   {
      return jointLimitUpper;
   }

   public void setJointLimitUpper(double jointLimitUpper)
   {
      this.jointLimitUpper = jointLimitUpper;
   }
   
   public double getVelocityLimitUpper()
   {
      return velocityLimitUpper;
   }
   
   public double getVelocityLimitLower()
   {
      return velocityLimitLower;
   }
   
   public void setVelocityLimit(double velocityLimitLower, double velocityLimitUpper)
   {
      this.velocityLimitLower = velocityLimitLower;
      this.velocityLimitUpper = velocityLimitUpper;
   }

   public void setEffortLimit(double effortLimit)
   {
      setEffortLimits(-effortLimit, effortLimit);
   }

   public void setEffortLimits(double effortLimitLower, double effortLimitUpper)
   {
      this.effortLimitLower = effortLimitLower;
      this.effortLimitUpper = effortLimitUpper;
   }

   public double getMinEffortLimit()
   {
      return effortLimitLower;
   }

   public double getMaxEffortLimit()
   {
      return effortLimitUpper;
   }

   @Deprecated
   public boolean getIntegrateDesiredAccelerations()
   {
      return integrateDesiredAccelerations;
   }

   @Deprecated
   public void setIntegrateDesiredAccelerations(boolean integrateDesiredAccelerations)
   {
      this.integrateDesiredAccelerations = integrateDesiredAccelerations;
   }

   @Deprecated
   public boolean getResetDesiredAccelerationIntegrator()
   {
      boolean ret = resetDesiredAccelerationIntegrator;
      resetDesiredAccelerationIntegrator = false;
      return ret;
   }

   @Deprecated
   public void resetDesiredAccelerationIntegrator()
   {
      resetDesiredAccelerationIntegrator = true;
   }

   @Deprecated
   public boolean getResetIntegrator()
   {
      boolean ret = resetIntegrator;
      resetIntegrator = false;
      return ret;
   }

   @Deprecated
   public void resetIntegrator()
   {
      resetIntegrator = true;
   }

   
   public double getqDesired()
   {
      return qDesired;
   }

   
   public double getQdDesired()
   {
      return qdDesired;
   }

   @Deprecated
   public double getKp()
   {
      return kp;
   }

   @Deprecated
   public double getKd()
   {
      return kd;
   }

   public abstract boolean isPassiveJoint();

   /*
    * VRC HACKS
    */

   public void setqDesired(double qDesired)
   {
      this.qDesired = qDesired;
   }

   public void setQdDesired(double qdDesired)
   {
      this.qdDesired = qdDesired;
   }

   @Deprecated
   public void setKp(double kp)
   {
      this.kp = kp;
   }

   @Deprecated
   public void setKd(double kd)
   {
      this.kd = kd;
   }

   @Override
   public void calculateJointStateChecksum(GenericCRC32 checksum)
   {
      checksum.update(q);
      checksum.update(qd);
      checksum.update(qdd);
   }


   public abstract FrameVector3D getJointAxis();

   public abstract void getJointAxis(FrameVector3D axisToPack);

   // DRC Hack

   @Deprecated
   public boolean isUnderPositionControl()
   {
      return isUnderPositionControl;
   }

   @Deprecated
   public void setUnderPositionControl(boolean val)
   {
      isUnderPositionControl = val;
   }

   public double getTauMeasured()
   {
      return tauMeasured;
   }

   public void setTauMeasured(double tauMeasured)
   {
      this.tauMeasured = tauMeasured;
   }

   public boolean isEnabled()
   {
      return enabled;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public boolean isOnline()
   {
      return isOnline;
   }

   public void setOnline(boolean isOnline)
   {
      this.isOnline = isOnline;
   }

   @Deprecated
   public boolean isUseFeedBackForceControl()
   {
      return useFeedBackForceControl;
   }

   @Deprecated
   public void setUseFeedBackForceControl(boolean useFeedBackForceControl)
   {
      this.useFeedBackForceControl = useFeedBackForceControl;
   }
}