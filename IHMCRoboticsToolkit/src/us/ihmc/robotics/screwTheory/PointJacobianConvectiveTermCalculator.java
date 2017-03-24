package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;

/**
 * @author twan
 *         Date: 5/19/13
 */
public class PointJacobianConvectiveTermCalculator
{
   private final TwistCalculator twistCalculator;
   private final Twist twist = new Twist();
   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final ConvectiveTermCalculator convectiveTermCalculator = new ConvectiveTermCalculator();
   private final FramePoint bodyFixedPoint = new FramePoint();
//   private final FrameVector bodyFixedPointVelocity = new FrameVector();
//   private final FrameVector tempVector = new FrameVector();



   public PointJacobianConvectiveTermCalculator(TwistCalculator twistCalculator)
   {
      this.twistCalculator = twistCalculator;
   }

   public void compute(PointJacobian pointJacobian, FrameVector pPointVelocity)
   {
      GeometricJacobian jacobian = pointJacobian.getGeometricJacobian();
      bodyFixedPoint.setIncludingFrame(pointJacobian.getPoint());
      bodyFixedPoint.changeFrame(jacobian.getBaseFrame());

      twistCalculator.getRelativeTwist(jacobian.getBase(), jacobian.getEndEffector(), twist);
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
