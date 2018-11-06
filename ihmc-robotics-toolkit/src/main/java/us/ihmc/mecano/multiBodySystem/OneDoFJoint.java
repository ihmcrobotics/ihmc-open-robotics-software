package us.ihmc.mecano.multiBodySystem;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.screwTheory.GenericCRC32;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public abstract class OneDoFJoint extends AbstractInverseDynamicsJoint
{
   protected Twist unitJointTwist;
   protected Twist unitSuccessorTwist;
   protected Twist unitPredecessorTwist;

   protected SpatialAcceleration unitJointAcceleration;
   protected SpatialAcceleration unitSuccessorAcceleration;
   protected SpatialAcceleration unitPredecessorAcceleration;

   private double q;
   private double qd;
   private double qdd;
   private double qddDesired;

   private double tau;

   private double effortLimitLower = Double.NEGATIVE_INFINITY;
   private double effortLimitUpper = Double.POSITIVE_INFINITY;

   private double jointLimitLower = Double.NEGATIVE_INFINITY;
   private double jointLimitUpper = Double.POSITIVE_INFINITY;
   private double velocityLimitLower = Double.NEGATIVE_INFINITY;
   private double velocityLimitUpper = Double.POSITIVE_INFINITY;

   public OneDoFJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent)
   {
      super(name, predecessor, transformToParent);
   }

   @Override
   public void getJointTwist(Twist twistToPack)
   {
      twistToPack.setIncludingFrame(unitJointTwist);
      twistToPack.scale(qd);
   }

   @Override
   public void getSuccessorTwist(Twist twistToPack)
   {
      twistToPack.setIncludingFrame(unitSuccessorTwist);
      twistToPack.scale(qd);
   }

   @Override
   public void getPredecessorTwist(Twist twistToPack)
   {
      twistToPack.setIncludingFrame(unitPredecessorTwist);
      twistToPack.scale(qd);
   }

   @Override
   public void getJointAcceleration(SpatialAcceleration accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(unitJointAcceleration);
      accelerationToPack.scale(qdd);
   }

   @Override
   public void getSuccessorAcceleration(SpatialAcceleration accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(unitSuccessorAcceleration);
      accelerationToPack.scale(qdd);
   }

   @Override
   public void getDesiredJointAcceleration(SpatialAcceleration accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(unitJointAcceleration);
      accelerationToPack.scale(qddDesired);
   }

   @Override
   public void getDesiredSuccessorAcceleration(SpatialAcceleration accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(unitSuccessorAcceleration);
      accelerationToPack.scale(qddDesired);
   }

   @Override
   public void getDesiredPredecessorAcceleration(SpatialAcceleration accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(unitPredecessorAcceleration);
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
      unitSuccessorTwist.getReferenceFrame().checkReferenceFrameMatch(jointWrench.getReferenceFrame());
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
      twistToPack.setIncludingFrame(unitJointTwist);
   }

   public void getUnitJointAcceleration(SpatialAcceleration accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(unitJointAcceleration);
   }

   /**
    * Setup the motion subspace for this one DoF joint assuming that the field
    * {@code unitSuccessorTwist} has already been properly setup.
    */
   protected void setMotionSubspace()
   {
      this.motionSubspace = new GeometricJacobian(this, unitSuccessorTwist.getReferenceFrame());
      this.motionSubspace.compute();
   }

   @Override
   public void getUnitTwist(int dofIndex, Twist unitTwistToPack)
   {
      if (dofIndex != 0)
         throw new ArrayIndexOutOfBoundsException("Illegal index: " + dofIndex + ", was expecting dofIndex equal to 0.");
      unitTwistToPack.setIncludingFrame(unitSuccessorTwist);
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

   public OneDoFJoint checkAndGetAsOneDoFJoint(JointBasics originalJoint)
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
   public void setJointPositionVelocityAndAcceleration(JointBasics originalJoint)
   {
      OneDoFJoint oneDoFOriginalJoint = checkAndGetAsOneDoFJoint(originalJoint);
      setQ(oneDoFOriginalJoint.getQ());
      setQd(oneDoFOriginalJoint.getQd());
      setQdd(oneDoFOriginalJoint.getQdd());
   }

   @Override
   public void setQddDesired(JointBasics originalJoint)
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

   public double getVelocityLimitLower()
   {
      return velocityLimitLower;
   }

   public double getVelocityLimitUpper()
   {
      return velocityLimitUpper;
   }

   public void setVelocityLimits(double velocityLimitLower, double velocityLimitUpper)
   {
      this.velocityLimitLower = velocityLimitLower;
      this.velocityLimitUpper = velocityLimitUpper;
   }

   public void setVelocityLimit(double velocityLimit)
   {
      setVelocityLimits(-velocityLimit, velocityLimit);
   }

   public void setEffortLimits(double effortLimitLower, double effortLimitUpper)
   {
      this.effortLimitLower = effortLimitLower;
      this.effortLimitUpper = effortLimitUpper;
   }

   public void setEffortLimit(double effortLimit)
   {
      setEffortLimits(-effortLimit, effortLimit);
   }

   public double getEffortLimitLower()
   {
      return effortLimitLower;
   }

   public double getEffortLimitUpper()
   {
      return effortLimitUpper;
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
}