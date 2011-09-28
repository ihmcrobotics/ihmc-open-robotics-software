package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.Twist;

public class BodyVelocityEstimatorScrewTheory implements BodyVelocityEstimator
{
   private final ReferenceFrame footZUpFrame;
   private final RigidBody foot;
   private final SixDoFJoint imuJoint;
   private final Twist bodyTwist = new Twist();
   private final FrameVector bodyLinearVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempLinearPart = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempAngularPart = new FrameVector(ReferenceFrame.getWorldFrame());
   private final RobotSide robotSide;

   public BodyVelocityEstimatorScrewTheory(RigidBody foot, ReferenceFrame footZUpFrame, SixDoFJoint imuJoint, RobotSide robotSide)
   {
      this.robotSide = robotSide;
      this.foot = foot;
      this.footZUpFrame = footZUpFrame;
      this.imuJoint = imuJoint;
   }
   
   public void estimateBodyVelocity()
   {
      packTwistOfAnkleWithRespectToIMU(bodyTwist, robotSide);
      tempLinearPart.setToZero(bodyTwist.getExpressedInFrame());
      bodyTwist.packLinearPart(tempLinearPart.getVector());

      packAngularVelocityOfAnkleZUpWithRespectToIMU(tempAngularPart, robotSide);
      bodyTwist.set(footZUpFrame, bodyTwist.getBaseFrame(), bodyTwist.getExpressedInFrame(), tempLinearPart.getVector(), tempAngularPart.getVector());
      
      bodyTwist.invert();
      bodyTwist.changeFrame(imuJoint.getFrameAfterJoint());
      bodyLinearVelocity.setToZero(bodyTwist.getExpressedInFrame());
      bodyTwist.packLinearPart(bodyLinearVelocity.getVector());
      bodyLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
   }
   
   public void packBodyVelocity(FrameVector bodyVelocityToPack)
   {
      bodyVelocityToPack.setAndChangeFrame(this.bodyLinearVelocity);
   }

   private final Twist jointTwist = new Twist(); 
   private void packTwistOfAnkleWithRespectToIMU(Twist twistToPack, RobotSide robotSide)
   {
      RigidBody currentBody = foot;
      ReferenceFrame footFrame = foot.getParentJoint().getFrameAfterJoint();
      twistToPack.setToZero(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), footFrame);

      while (currentBody != imuJoint.getSuccessor())
      {
         currentBody.getParentJoint().packPredecessorTwist(jointTwist);
         jointTwist.changeFrame(twistToPack.getExpressedInFrame());
         twistToPack.add(jointTwist);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }
      twistToPack.invert();
      twistToPack.changeBodyFrameNoRelativeTwist(footFrame);
      twistToPack.changeBaseFrameNoRelativeTwist(imuJoint.getFrameAfterJoint());
   }
   
   private void packAngularVelocityOfAnkleZUpWithRespectToIMU(FrameVector angularVelocityToPack, RobotSide robotSide)
   {
      ReferenceFrame footFrame = foot.getParentJoint().getFrameAfterJoint();

      imuJoint.packJointTwist(jointTwist);
      angularVelocityToPack.setToZero(jointTwist.getExpressedInFrame());
      jointTwist.packAngularPart(angularVelocityToPack.getVector()); // angular velocity of IMU w.r.t world == IMU w.r.t ankle ZUp by assumption, in IMU frame
      angularVelocityToPack.changeFrame(footFrame); // angular velocity of IMU w.r.t. ankle ZUp, in foot frame
      angularVelocityToPack.scale(-1.0); // angular velocity of ankle ZUp w.r.t. IMU, in foot frame
   }
}
