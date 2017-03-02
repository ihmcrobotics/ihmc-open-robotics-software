package us.ihmc.quadrupedRobotics.mechanics.inverseKinematics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

/**
 * This inverse kinematics calculator requires the kinematic chain for the leg to follow a Roll Pitch Pitch model.
 * @author steel
 *
 */
public class QuadrupedLegThreeDoFClosedFormInverseKinematicsCalculator
{
   private final Vector3D offsetFromHipRollToHipPitch = new Vector3D();
   private final double thighLength;
   private final double shinLength;
   private final double hipThetaOffset, kneeThetaOffset;

   private final RigidBodyTransform transformFromBeforeHipRollToAfterHipRoll = new RigidBodyTransform();
   private final Vector3D footPositionInFrameAfterHipRoll = new Vector3D();
   private final Vector3D footPositionInFrameBeforeHipPitch = new Vector3D();

   private boolean bendKneesIn = false;

   /**
    *
    * @param offsetFromHipRollToHipPitch
    * @param referenceFrames
    * @param robotQuadrant
    */
   public QuadrupedLegThreeDoFClosedFormInverseKinematicsCalculator(Vector3D offsetFromHipRollToHipPitch, QuadrupedReferenceFrames referenceFrames, RobotQuadrant robotQuadrant)
   {
      this.offsetFromHipRollToHipPitch.set(offsetFromHipRollToHipPitch);
      ReferenceFrame hipPitchFrame = referenceFrames.getHipPitchFrame(robotQuadrant);
      ReferenceFrame kneePitchFrame = referenceFrames.getKneeFrame(robotQuadrant);
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);

      FramePoint hipPitch = new FramePoint(hipPitchFrame);
      FramePoint kneePitch = new FramePoint(kneePitchFrame);
      FramePoint foot = new FramePoint(footFrame);

      kneePitch.changeFrame(hipPitchFrame);

      double hipToKneeTheta = Math.atan2(kneePitch.getX(), -kneePitch.getZ());
      hipThetaOffset = hipToKneeTheta;

//    System.out.println("hipThetaOffset = " + hipThetaOffset);

      thighLength = kneePitch.distance(hipPitch);

      kneePitch.changeFrame(kneePitchFrame);
      foot.changeFrame(kneePitchFrame);
      shinLength = foot.distance(kneePitch);

      if(foot.getX() != 0)
      {
         double kneeToFootTheta = Math.atan(foot.getZ() / foot.getX());
         kneeThetaOffset = Math.PI / 2 - kneeToFootTheta;
      }
      else
      {
         kneeThetaOffset = 0.0;
      }

//      System.out.println("kneeThetaOffset = " + kneeThetaOffset);

   }

   public static QuadrupedLegThreeDoFClosedFormInverseKinematicsCalculator createFromLegAttachmentFrame(RobotQuadrant robotQuadrant, QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties)
   {
      //make these here to ensure we get a zero pose
      FullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);

      ReferenceFrame legAttachmentFrame = referenceFrames.getLegAttachmentFrame(robotQuadrant);
      ReferenceFrame frameBeforeHipPitch = referenceFrames.getHipPitchFrame(robotQuadrant).getParent();

      FramePoint offsetFromHipRollToHipPitch = new FramePoint(frameBeforeHipPitch);
      offsetFromHipRollToHipPitch.changeFrame(legAttachmentFrame);

      return new QuadrupedLegThreeDoFClosedFormInverseKinematicsCalculator(offsetFromHipRollToHipPitch.getVectorCopy(), referenceFrames, robotQuadrant);
   }

   public static QuadrupedLegThreeDoFClosedFormInverseKinematicsCalculator createFromHipRollFrame(RobotQuadrant robotQuadrant, QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties)
   {
      //make these here to ensure we get a zero pose
      FullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);

      ReferenceFrame frameAfterHipRoll = referenceFrames.getFrameBeforeLegJoint(robotQuadrant, LegJointName.HIP_ROLL);
      ReferenceFrame frameBeforeHipPitch = referenceFrames.getHipPitchFrame(robotQuadrant).getParent();

      FramePoint offsetFromHipRollToHipPitch = new FramePoint(frameBeforeHipPitch);
      offsetFromHipRollToHipPitch.changeFrame(frameAfterHipRoll);

      return new QuadrupedLegThreeDoFClosedFormInverseKinematicsCalculator(offsetFromHipRollToHipPitch.getVectorCopy(), referenceFrames, robotQuadrant);
   }

   public void setBendKneesIn(boolean bendKneesIn)
   {
      this.bendKneesIn = bendKneesIn;
   }

   public boolean computeJointAnglesGivenFootInFrameBeforeHipRoll(Vector3D footPositionInFrameBeforeHipRoll, double[] jointAnglesToPack)
   {
      // hipRollAngle will be guaranteed to be between +-PI since using atan2 here.
      double hipRollAngle = Math.atan2(footPositionInFrameBeforeHipRoll.getY(), -footPositionInFrameBeforeHipRoll.getZ());

      transformFromBeforeHipRollToAfterHipRoll.setIdentity();
      transformFromBeforeHipRollToAfterHipRoll.setRotationRollAndZeroTranslation(-hipRollAngle);

      footPositionInFrameAfterHipRoll.set(footPositionInFrameBeforeHipRoll);
      transformFromBeforeHipRollToAfterHipRoll.transform(footPositionInFrameAfterHipRoll);

//    if (Math.abs(footPositionInFrameAfterHipRoll.getY()) > 1e-7) throw new RuntimeException("Oops. footPositionInFrameAfterHipRoll.getY() = " + footPositionInFrameAfterHipRoll.getY());

      footPositionInFrameBeforeHipPitch.set(footPositionInFrameAfterHipRoll);
      footPositionInFrameBeforeHipPitch.sub(offsetFromHipRollToHipPitch);

      double lengthFromHipToFooSquared = footPositionInFrameBeforeHipPitch.lengthSquared();

      double numerator = lengthFromHipToFooSquared - thighLength * thighLength - shinLength * shinLength;
      double denominator = 2.0 * thighLength * shinLength;

      boolean valid = true;

      // One invalid solution is that it can't reach far enough.
      double ratio = numerator / denominator;
      if (Math.abs(ratio) > 1.0 + 1e-7)
      {
         valid = false;
      }

      ratio = limitToPlusMinusOne(ratio);

      // kneeAngle will always be positive, between 0.0 and PI here since using acos. If you want the other solution, use a negative here.
      double kneeAngle = Math.acos(ratio);

      if (bendKneesIn)
      {
         kneeAngle = -1.0 * kneeAngle;
      }

      double lengthFromHipToFoot = Math.sqrt(lengthFromHipToFooSquared);
      double theta0 = -Math.atan2(footPositionInFrameBeforeHipPitch.getX(), -footPositionInFrameBeforeHipPitch.getZ());

      double sineKneeAngle = shinLength / lengthFromHipToFoot * Math.sin(kneeAngle);

      // Mathematically, these checks for +-1.0 bounds should not be necessary, but due to numerical round off, they might be.
      sineKneeAngle = limitToPlusMinusOne(sineKneeAngle);

      double theta1 = -Math.asin(sineKneeAngle);

      double hipPitchAngle = theta0 + theta1;

      jointAnglesToPack[0] = hipRollAngle;
      jointAnglesToPack[1] = hipPitchAngle + hipThetaOffset;
      jointAnglesToPack[2] = kneeAngle - hipThetaOffset - kneeThetaOffset ;

      return valid;
   }

   public double getKneeAngleAtMaxLength()
   {
	   return  -hipThetaOffset - kneeThetaOffset;
   }

   private double limitToPlusMinusOne(double input)
   {
      if (input > 1.0)
         return 1.0;
      if (input < -1.0)
         return -1.0;

      return input;
   }

}
