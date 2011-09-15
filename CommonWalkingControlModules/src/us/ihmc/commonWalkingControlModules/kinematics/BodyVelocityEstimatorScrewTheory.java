package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.Twist;

public class BodyVelocityEstimatorScrewTheory
{
   private final SideDependentList<ReferenceFrame> footZUpFrames;
   private final SideDependentList<RigidBody> feet;
   private final SixDoFJoint imuJoint;
   private final SideDependentList<Twist> bodyTwists = new SideDependentList<Twist>();
   private final SideDependentList<FrameVector> bodyLinearVelocities = new SideDependentList<FrameVector>();
   private final FrameVector tempLinearPart = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempAngularPart = new FrameVector(ReferenceFrame.getWorldFrame());

   public BodyVelocityEstimatorScrewTheory(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> footZUpFrames, SixDoFJoint imuJoint)
   {
      this.feet = feet;
      this.footZUpFrames = footZUpFrames;
      this.imuJoint = imuJoint;
      for (RobotSide robotSide : RobotSide.values())
      {
         bodyTwists.put(robotSide, new Twist());
         bodyLinearVelocities.put(robotSide, new FrameVector(ReferenceFrame.getWorldFrame()));
      }
   }
   
   public void estimateBodyVelocity()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         Twist bodyTwist = bodyTwists.get(robotSide);
         packTwistOfAnkleWithRespectToIMU(bodyTwist, robotSide);
         tempLinearPart.setToZero(bodyTwist.getExpressedInFrame());
         bodyTwist.packLinearPart(tempLinearPart.getVector());

         packAngularVelocityOfAnkleZUpWithRespectToIMU(tempAngularPart, robotSide);
         bodyTwist.set(footZUpFrames.get(robotSide), bodyTwist.getBaseFrame(), bodyTwist.getExpressedInFrame(), tempLinearPart.getVector(), tempAngularPart.getVector());
         
         bodyTwist.invert();
         bodyTwist.changeFrame(imuJoint.getFrameAfterJoint());
         FrameVector bodyLinearVelocity = bodyLinearVelocities.get(robotSide);
         bodyLinearVelocity.setToZero(bodyTwist.getExpressedInFrame());
         bodyTwist.packLinearPart(bodyLinearVelocity.getVector());
         bodyLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      }
   }
   
   public FrameVector getBodyVelocity(RobotSide robotSide)
   {
      return bodyLinearVelocities.get(robotSide);
   }

   private final Twist jointTwist = new Twist(); 
   private void packTwistOfAnkleWithRespectToIMU(Twist twistToPack, RobotSide robotSide)
   {
      RigidBody foot = feet.get(robotSide);
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
      RigidBody foot = feet.get(robotSide);
      ReferenceFrame footFrame = foot.getParentJoint().getFrameAfterJoint();

      imuJoint.packJointTwist(jointTwist);
      angularVelocityToPack.setToZero(jointTwist.getExpressedInFrame());
      jointTwist.packAngularPart(angularVelocityToPack.getVector()); // angular velocity of IMU w.r.t world == IMU w.r.t ankle ZUp by assumption, in IMU frame
      angularVelocityToPack.changeFrame(footFrame); // angular velocity of IMU w.r.t. ankle ZUp, in foot frame
      angularVelocityToPack.scale(-1.0); // angular velocity of ankle ZUp w.r.t. IMU, in foot frame
   }
}
