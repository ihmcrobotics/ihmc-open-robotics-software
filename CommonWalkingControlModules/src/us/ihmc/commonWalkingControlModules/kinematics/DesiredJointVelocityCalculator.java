package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;

public class DesiredJointVelocityCalculator
{
   private final ProcessedSensorsInterface processedSensors;
   private final SwingFullLegJacobian swingFullLegJacobian;
   private final ReferenceFrame swingFootFrame;

   public DesiredJointVelocityCalculator(ProcessedSensorsInterface processedSensors, CommonHumanoidReferenceFrames referenceFrames,
           SwingFullLegJacobian swingLegJacobian)
   {
      this.processedSensors = processedSensors;
      this.swingFullLegJacobian = swingLegJacobian;

      this.swingFootFrame = swingLegJacobian.getGeometricJacobian().getEndEffectorFrame();
   }

   public void packDesiredJointVelocities(LegJointVelocities legJointVelocitiesToPack, Twist desiredTwistOfSwingFootWithRespectToWorld, double alpha)
   {
      // defensive copy, change frame
      desiredTwistOfSwingFootWithRespectToWorld = new Twist(desiredTwistOfSwingFootWithRespectToWorld);

//    desiredTwistOfSwingFoot_World.changeFrame(bodyFrame);

      // compute jacobian
      swingFullLegJacobian.computeJacobian();

      // compute twist of world with respect to body
      Twist twistOfWorldWithRespectToBody = processedSensors.getTwistOfPelvisWithRespectToWorld();    // twist of pelvis w.r.t. world at this point
      twistOfWorldWithRespectToBody.invert();    // twist of world w.r.t. body at this point.
      twistOfWorldWithRespectToBody.changeFrame(desiredTwistOfSwingFootWithRespectToWorld.getExpressedInFrame());

      // compute twist of swing foot with respect to pelvis
      Twist twistOfSwingFootWithRespectToBody = new Twist(twistOfWorldWithRespectToBody);    // twist of world with respect to body at this point
      twistOfSwingFootWithRespectToBody.add(desiredTwistOfSwingFootWithRespectToWorld);    // twist of swing foot w.r.t. body at this point
      twistOfSwingFootWithRespectToBody.changeFrame(swingFootFrame);

      // compute joint velocities
      swingFullLegJacobian.packJointVelocitiesGivenTwist(legJointVelocitiesToPack, twistOfSwingFootWithRespectToBody, alpha);
   }

   public double swingFullLegJacobianDeterminant()
   {
      return swingFullLegJacobian.det();
   }
}
