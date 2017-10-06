package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;

/**
 * @author twan
 *         Date: 5/19/13
 */
public class PointJacobianConvectiveTermCalculator
{
   private final Twist twist = new Twist();
   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final ConvectiveTermCalculator convectiveTermCalculator = new ConvectiveTermCalculator();
   private final FramePoint3D bodyFixedPoint = new FramePoint3D();
//   private final FrameVector bodyFixedPointVelocity = new FrameVector();
//   private final FrameVector tempVector = new FrameVector();

   public void compute(PointJacobian pointJacobian, FrameVector3D pPointVelocity)
   {
      GeometricJacobian jacobian = pointJacobian.getGeometricJacobian();
      bodyFixedPoint.setIncludingFrame(pointJacobian.getPoint());
      bodyFixedPoint.changeFrame(jacobian.getBaseFrame());

      jacobian.getEndEffector().getBodyFixedFrame().getTwistRelativeToOther(jacobian.getBaseFrame(), twist);
      convectiveTermCalculator.computeJacobianDerivativeTerm(jacobian, convectiveTerm);
      convectiveTerm.changeFrame(jacobian.getBaseFrame(), twist, twist);

      twist.changeFrame(jacobian.getBaseFrame());
      convectiveTerm.getAccelerationOfPointFixedInBodyFrame(twist, bodyFixedPoint, pPointVelocity);

//      bodyFixedPointVelocity.setToZero(jacobian.getBaseFrame());
//      twist.packVelocityOfPointFixedInBodyFrame(bodyFixedPointVelocity, bodyFixedPoint);
//
//      twist.packAngularPart(tempVector);
//      tempVector.cross(tempVector, bodyFixedPointVelocity);
//      pPointVelocity.add(tempVector);
   }
}
