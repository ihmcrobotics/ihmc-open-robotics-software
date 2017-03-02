package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DesiredJointAccelerationCalculatorOld
{
   private final GeometricJacobian jacobian;
   private final DenseMatrix64F jacobianDerivativeTermMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F jointAccelerations;
   private final DenseMatrix64F biasedSpatialAcceleration = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final SpatialAccelerationVector jacobianDerivativeTerm = new SpatialAccelerationVector();

   private final SpatialAccelerationVector zeroAcceleration = new SpatialAccelerationVector();
   private final Twist twistOfCurrentWithRespectToBase = new Twist();
   private final Twist jointTwist = new Twist();
   private final int sign;
   private final JacobianSolver jacobianSolver;

   public DesiredJointAccelerationCalculatorOld(GeometricJacobian jacobian, JacobianSolver jacobianSolver)
   {
      this.jointAccelerations = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jacobian.getJointsInOrder()), 1);
      this.sign = ScrewTools.isAncestor(jacobian.getEndEffector(), jacobian.getBase()) ? 1 : -1; // because the sign of a joint acceleration changes when you switch a joint's 'direction' in the chain 
      this.jacobian = jacobian;
      this.jacobianSolver = jacobianSolver;
   }

   /**
    * Sets the accelerations for the RevoluteJoints in legJoints
    * Assumes that the swingLegJacobian is already updated
    * Assumes that the rootJoint's acceleration has already been set
    */
   public void compute(SpatialAccelerationVector accelerationOfEndEffectorWithRespectToBase)
   {
      computeJacobianDerivativeTerm(jacobianDerivativeTerm);
      computeJointAccelerations(accelerationOfEndEffectorWithRespectToBase, jacobianDerivativeTerm);
   }

   public void computeJacobianDerivativeTerm(SpatialAccelerationVector accelerationToPack)
   {
      ReferenceFrame endEffectorFrame = jacobian.getEndEffectorFrame();
      twistOfCurrentWithRespectToBase.setToZero(endEffectorFrame, endEffectorFrame, endEffectorFrame);
      accelerationToPack.setToZero(endEffectorFrame, endEffectorFrame, endEffectorFrame);
      RigidBody currentBody = jacobian.getEndEffector();
      for (int i = jacobian.getJointsInOrder().length - 1; i >= 0; i--)
      {
         twistOfCurrentWithRespectToBase.changeFrame(currentBody.getBodyFixedFrame());
         InverseDynamicsJoint joint = jacobian.getJointsInOrder()[i];
         if (currentBody == joint.getPredecessor())
         {
            joint.getPredecessorTwist(jointTwist);
            currentBody = joint.getSuccessor();
         }
         else
         {
            joint.getSuccessorTwist(jointTwist);
            currentBody = joint.getPredecessor();            
         }

         zeroAcceleration.setToZero(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), jointTwist.getExpressedInFrame());
         zeroAcceleration.changeFrame(endEffectorFrame, twistOfCurrentWithRespectToBase, jointTwist);
         zeroAcceleration.add(accelerationToPack);
         accelerationToPack.set(zeroAcceleration);

         jointTwist.invert();
         twistOfCurrentWithRespectToBase.add(jointTwist);
      }
      accelerationToPack.changeBodyFrameNoRelativeAcceleration(endEffectorFrame);
   }

   private void computeJointAccelerations(SpatialAccelerationVector accelerationOfEndEffectorWithRespectToBase, SpatialAccelerationVector jacobianDerivativeTerm)
   {
      accelerationOfEndEffectorWithRespectToBase.getBodyFrame().checkReferenceFrameMatch(jacobianDerivativeTerm.getBodyFrame());
      accelerationOfEndEffectorWithRespectToBase.getBaseFrame().checkReferenceFrameMatch(jacobianDerivativeTerm.getBaseFrame());
      accelerationOfEndEffectorWithRespectToBase.getExpressedInFrame().checkReferenceFrameMatch(jacobianDerivativeTerm.getExpressedInFrame());
      jacobian.getJacobianFrame().checkReferenceFrameMatch(accelerationOfEndEffectorWithRespectToBase.getExpressedInFrame());

      accelerationOfEndEffectorWithRespectToBase.getMatrix(biasedSpatialAcceleration, 0);    // unbiased at this point
      jacobianDerivativeTerm.getMatrix(jacobianDerivativeTermMatrix, 0);
      CommonOps.subtractEquals(biasedSpatialAcceleration, jacobianDerivativeTermMatrix);
      jacobianSolver.setJacobian(jacobian.getJacobianMatrix());
      jacobianSolver.solve(jointAccelerations, biasedSpatialAcceleration);

      CommonOps.scale(sign, jointAccelerations);
      ScrewTools.setDesiredAccelerations(jacobian.getJointsInOrder(), jointAccelerations);
   }
}
