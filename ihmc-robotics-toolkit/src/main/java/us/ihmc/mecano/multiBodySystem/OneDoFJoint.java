package us.ihmc.mecano.multiBodySystem;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
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
   public void setJointTau(int rowStart, DenseMatrix64F matrix)
   {
      setTau(matrix.get(rowStart, 0));
   }

   @Override
   public void getJointTau(int rowStart, DenseMatrix64F matrix)
   {
      MathTools.checkIntervalContains(matrix.getNumRows(), 1, Integer.MAX_VALUE);
      MathTools.checkIntervalContains(matrix.getNumCols(), 1, Integer.MAX_VALUE);
      matrix.set(0, 0, tau);
   }

   @Override
   public void getJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      MathTools.checkIntervalContains(matrix.getNumRows(), 1, Integer.MAX_VALUE);
      MathTools.checkIntervalContains(matrix.getNumCols(), 1, Integer.MAX_VALUE);
      matrix.set(rowStart, 0, qd);
   }

   @Override
   public void getJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      MathTools.checkIntervalContains(matrix.getNumRows(), 1, Integer.MAX_VALUE);
      MathTools.checkIntervalContains(matrix.getNumCols(), 1, Integer.MAX_VALUE);
      matrix.set(rowStart, 0, qdd);
   }

   @Override
   public void setJointAccelerationToZero()
   {
      qdd= 0.0;
   }

   @Override
   public void updateMotionSubspace()
   {
      // empty
   }

   @Override
   public void setJointWrench(Wrench jointWrench)
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
   public void setJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      setQdd(matrix.get(rowStart + 0, 0));
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
   public void getJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      matrix.set(rowStart, q);
   }

   @Override
   public void setJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      setQ(matrix.get(rowStart, 0));
   }

   @Override
   public void setJointVelocity(int rowStart, DenseMatrix64F matrix)
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

   public abstract FrameVector3DReadOnly getJointAxis();
}