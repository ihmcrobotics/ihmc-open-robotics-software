package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Computes Jacobian derivative times velocity vector Useful for dealing with task space
 * accelerations, since:
 *
 * J * v = T
 *
 * differentiate: \dot{J} * v + J * \dot{v} = \dot{T}
 *
 * This class computes \dot{J} * v (the convective term in this equation)
 */
public class ConvectiveTermCalculator
{
   private final SpatialAccelerationVector zeroAcceleration = new SpatialAccelerationVector();
   private final Twist twistOfCurrentWithRespectToBase = new Twist();
   private final Twist jointTwist = new Twist();

   public void computeJacobianDerivativeTerm(GeometricJacobian jacobian, SpatialAccelerationVector accelerationToPack)
   {
      ReferenceFrame endEffectorFrame = jacobian.getEndEffectorFrame();
      twistOfCurrentWithRespectToBase.setToZero(endEffectorFrame, endEffectorFrame, endEffectorFrame);
      accelerationToPack.setToZero(endEffectorFrame, endEffectorFrame, endEffectorFrame);
      RigidBody currentBody = jacobian.getEndEffector();
      InverseDynamicsJoint[] jointPathFromBaseToEndEffector = jacobian.getJointPathFromBaseToEndEffector();
      for (int i = jointPathFromBaseToEndEffector.length - 1; i >= 0; i--)
      {
         twistOfCurrentWithRespectToBase.changeFrame(currentBody.getBodyFixedFrame());
         InverseDynamicsJoint joint = jointPathFromBaseToEndEffector[i];
         if (currentBody == joint.getPredecessor())
         {
            joint.getPredecessorTwist(jointTwist);
            currentBody = joint.getSuccessor();
         }
         else if (currentBody == joint.getSuccessor())
         {
            joint.getSuccessorTwist(jointTwist);
            currentBody = joint.getPredecessor();
         }
         else
         {
            String errorMessage = "ConvectiveTermCalculator: Joints of jacobian are not sequential. Crashing.";
            throw new RuntimeException(errorMessage);
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
}
