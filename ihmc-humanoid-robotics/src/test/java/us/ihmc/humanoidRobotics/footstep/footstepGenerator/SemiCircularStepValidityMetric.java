package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import static org.junit.Assert.assertTrue;

import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepValidityMetric;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class SemiCircularStepValidityMetric implements FootstepValidityMetric
{
   private final static boolean DEBUG = false;
   private final double circleCenterOffset;
   private final double validRegionChordOffset;
   private final boolean checkForValidRegionOfSemiCircle;
   private boolean checkForSwingThroughStance = true;

   private final double footTwistLimitInRadians;
   private final double footReachLimitInMeters;
   private final RigidBody leftFootBody;
   private boolean valid = true;

   public SemiCircularStepValidityMetric(RigidBody leftFootBody, double circleCenterOffsetFromStanceCenter, double footTwistLimitInRadians,
         double footReachLimitInMeters, double validRegionChordOffset)
   {
      this.circleCenterOffset = circleCenterOffsetFromStanceCenter;
      this.validRegionChordOffset = validRegionChordOffset;
      this.checkForValidRegionOfSemiCircle = true;

      this.footTwistLimitInRadians = footTwistLimitInRadians;
      this.footReachLimitInMeters = footReachLimitInMeters;
      this.leftFootBody = leftFootBody;
   }

   public SemiCircularStepValidityMetric(RigidBody leftFootBody, double circleCenterOffsetFromStanceCenter, double footTwistLimitInRadians,
         double footReachLimitInMeters, boolean checkForValidRegionOfSemiCircle)
   {
      this.circleCenterOffset = circleCenterOffsetFromStanceCenter;
      this.validRegionChordOffset = 0.0;
      this.checkForValidRegionOfSemiCircle = checkForValidRegionOfSemiCircle;

      this.footTwistLimitInRadians = footTwistLimitInRadians;
      this.footReachLimitInMeters = footReachLimitInMeters;
      this.leftFootBody = leftFootBody;
   }

   public SemiCircularStepValidityMetric(RigidBody leftFootBody, double circleCenterOffsetFromStanceCenter, double footTwistLimitInRadians,
         double footReachLimitInMeters)
   {
      this.circleCenterOffset = circleCenterOffsetFromStanceCenter;
      this.validRegionChordOffset = 0.0;
      this.checkForValidRegionOfSemiCircle = true;

      this.footTwistLimitInRadians = footTwistLimitInRadians;
      this.footReachLimitInMeters = footReachLimitInMeters;
      this.leftFootBody = leftFootBody;
   }

   @Override
   public boolean assertValid(Footstep swingStart, Footstep stance, Footstep swingEnd)
   {
      return checkIfValid(swingStart, stance, swingEnd, true);
   }

   @Override
   public boolean assertValid(String message, Footstep swingStart, Footstep stance, Footstep swingEnd)
   {
      return checkIfValid(message, swingStart, stance, swingEnd, true);
   }

   @Override
   public boolean isValid(Footstep swingStart, Footstep stance, Footstep swingEnd)
   {
      return checkIfValid(swingStart, stance, swingEnd, false);
   }

   public boolean checkIfValid(Footstep swingStart, Footstep stance, Footstep swingEnd, boolean assertValidity)
   {
      return checkIfValid("", swingStart, stance, swingEnd, assertValidity);
   }

   public boolean checkIfValid(String message, Footstep swingStart, Footstep stance, Footstep swingEnd, boolean assertValidity)
   {
      assertValidIfTrue(message + " Previously Failed", assertValidity);
      assertValidIfTrue(message + " contact points requirement", assertValidity);
      checkStartStanceAndEndAlternateSides(message + " Alternating sides requirement", swingStart, stance, swingEnd, assertValidity);
      if (checkForValidRegionOfSemiCircle && checkForSwingThroughStance)
         checkIfSwingCrossesThroughStanceLeg(message, swingStart, stance, swingEnd, assertValidity);
      else
         checkFootstepIsInsideSemiCircle(message, stance, swingEnd, assertValidity);
      checkFootMeetsAngleRequirement(stance, swingEnd);
      assertValidIfTrue(message + " Angle requirement", assertValidity);

      return valid;
   }

   private void assertValidIfTrue(String message, boolean assertValidity)
   {
      if (assertValidity)
      {
         if (message != null)
            assertTrue(message, valid);
         else
            assertTrue(valid);
      }
   }

   private void checkFootMeetsAngleRequirement(Footstep stance, Footstep swingEnd)
   {
      FramePose3D stancePose = new FramePose3D();
      stance.getPose(stancePose);
      FramePose3D swingEndPose = new FramePose3D();
      swingEnd.getPose(swingEndPose);
      FrameOrientation2D stanceOrientation = new FrameOrientation2D(stancePose.getOrientation());
      FrameOrientation2D swingOrientation = new FrameOrientation2D(swingEndPose.getOrientation());
      double yawDifference = swingOrientation.difference(stanceOrientation);
      double allowedDifference = (Math.abs(yawDifference) - footTwistLimitInRadians) - 3e-16;
      if (allowedDifference > 0)
         valid = false;

   }

   private void checkStartStanceAndEndAlternateSides(String message, Footstep swingStart, Footstep stance, Footstep swingEnd, boolean assertValidity)
   {
      RobotSide swingStartSide = swingStart.getRobotSide();
      RobotSide stanceSide = stance.getRobotSide();
      RobotSide swingEndSide = swingEnd.getRobotSide();
      valid &= swingStartSide == swingEndSide;
      assertValidIfTrue(message + " swing step start and end must be same side", assertValidity);
      valid &= stanceSide == swingEndSide.getOppositeSide();
      assertValidIfTrue(message + " start and stance must be different", assertValidity);

   }

   private void checkFootstepIsInsideSemiCircle(String message, Footstep stance, Footstep swingEnd, boolean assertValidity)
   {
      // chord offset from bisector(diameter) needed separate from circular center offset!
      if (DEBUG)
         System.out.println("SemiCircularStepValidityMetric: stance = " + stance.toString() + " and swingEnd = " + swingEnd);
      FrameVector3D translationFromFootCenterToCircleCenter = getTranslationVector(stance, circleCenterOffset);
      FrameVector3D vectorToSwingEnd = offCenterVectorToSwingEnd(stance, swingEnd, translationFromFootCenterToCircleCenter);
      double magnitudeSquared = vectorToSwingEnd.lengthSquared();
      valid &= magnitudeSquared <= footReachLimitInMeters * footReachLimitInMeters;
      assertValidIfTrue(message + " In distance check", assertValidity);

      if (checkForValidRegionOfSemiCircle)
      {
         translationFromFootCenterToCircleCenter = getTranslationVector(stance, circleCenterOffset + validRegionChordOffset);
         vectorToSwingEnd = offCenterVectorToSwingEnd(stance, swingEnd, translationFromFootCenterToCircleCenter);
         translationFromFootCenterToCircleCenter = getTranslationVector(stance, 1.0);

         double dotProduct = vectorToSwingEnd.dot(translationFromFootCenterToCircleCenter);

         valid &= dotProduct >= 0;
         if (DEBUG)
            System.out.println("SemiCircularStepValidityMetric: dotProduct = " + dotProduct);
         assertValidIfTrue(message + " In check for correct side", assertValidity);

      }
   }

   private void checkIfSwingCrossesThroughStanceLeg(String message, Footstep swingStart, Footstep stance, Footstep swingEnd, boolean assertValidity)
   {
      boolean priorValidity = valid;
      checkFootstepIsInsideSemiCircle(message, stance, swingEnd, false);
      boolean valid1 = valid;
      valid = true;
      checkFootstepIsInsideSemiCircle(message, swingEnd, stance, false);
      boolean valid2 = valid;
      valid = true;

      FramePose3D initialFramePose = new FramePose3D();
      FramePose3D stanceFramePose = new FramePose3D();
      FramePose3D endingFramePose = new FramePose3D();
      swingStart.getPose(initialFramePose);
      stance.getPose(stanceFramePose);
      swingEnd.getPose(endingFramePose);

      FramePoint2D initialPoint = new FramePoint2D(initialFramePose.getPosition());
      FramePoint2D stancePoint = new FramePoint2D(stanceFramePose.getPosition());
      FramePoint2D endPoint = new FramePoint2D(endingFramePose.getPosition());
      boolean swingsThroughStanceLeg;
      if (initialPoint.epsilonEquals(endPoint, 1.0e-12))
      {
         swingsThroughStanceLeg = false;
      }
      else
      {
         FrameLineSegment2D swingLine = new FrameLineSegment2D(initialPoint, endPoint);
         double swingLineDistance = swingLine.distance(stancePoint);

         swingsThroughStanceLeg = checkIfSwingGoesThroughStanceLeg(stance, stanceFramePose, swingLine);

         swingsThroughStanceLeg |= swingLineDistance < circleCenterOffset;
      }

      valid = priorValidity && (valid1 || valid2) && !swingsThroughStanceLeg;
      assertValidIfTrue(message + " Check for footstep side acceptability and no swing through stance leg", assertValidity);
   }

   private boolean checkIfSwingGoesThroughStanceLeg(Footstep stance, FramePose3D stanceFramePose, FrameLineSegment2D swingLine)
   {
      RobotSide stanceFootstepSide = stance.getRobotSide();

      FrameVector3D offset1 = getPerpendicularOffset(stanceFootstepSide, stanceFramePose, circleCenterOffset);
      FrameVector3D offset2 = getPerpendicularOffset(stanceFootstepSide, stanceFramePose, -3 * footReachLimitInMeters);
      FramePoint2D stanceInsideOffset = new FramePoint2D(ReferenceFrame.getWorldFrame(), offset1.getX(), offset1.getY());
      FramePoint2D stanceOutsideOffset = new FramePoint2D(ReferenceFrame.getWorldFrame(), offset2.getX(), offset2.getY());
      FrameLineSegment2D crossThroughLine = new FrameLineSegment2D(stanceInsideOffset, stanceOutsideOffset);
      FramePoint2D intersectionPoint = crossThroughLine.intersectionWith(swingLine);
      boolean swingsThroughStanceLeg;
      if (intersectionPoint == null)
         swingsThroughStanceLeg = false;
      else
         swingsThroughStanceLeg = true;
      return swingsThroughStanceLeg;
   }

   private static FrameVector3D getPerpendicularOffset(RobotSide stanceFootstepSide, FramePose3D stanceFramePose, double semiCircleOffset)
   {
      double sideOffset = semiCircleOffset * ((stanceFootstepSide == RobotSide.LEFT) ? -1 : 1);
      double offsetDirAngle = stanceFramePose.getYaw() + Math.PI / 2;
      FrameVector3D offset = new FrameVector3D(ReferenceFrame.getWorldFrame(), new double[] { Math.cos(offsetDirAngle) * sideOffset,
            Math.sin(offsetDirAngle) * sideOffset, 0 });

      FramePoint3D stanceFramePosition = new FramePoint3D(stanceFramePose.getPosition());
      FrameVector3D offsetStart = new FrameVector3D(stanceFramePosition);
      offsetStart.add(offset);

      return offsetStart;
   }

   private FrameVector3D offCenterVectorToSwingEnd(Footstep stance, Footstep swingEnd, FrameVector3D translationFromFootCenterToCircleCenter)
   {
      FramePoint3D circleCenter;
      FramePose3D stanceSole = new FramePose3D();
      FramePose3D swingEndSole = new FramePose3D();
      stance.getPose(stanceSole);
      swingEnd.getPose(swingEndSole);

      circleCenter = new FramePoint3D(stanceSole.getPosition());
      circleCenter.add(translationFromFootCenterToCircleCenter);//change frame of translation first?

      FrameVector3D vectorToSwingEnd = new FrameVector3D(ReferenceFrame.getWorldFrame());
      vectorToSwingEnd.set(swingEndSole.getPosition());
      vectorToSwingEnd.sub(circleCenter);

      return vectorToSwingEnd;
   }

   private FrameVector3D getTranslationVector(Footstep stance, double circleCenterOffset)
   {
      RobotSide stanceSide = stance.getRobotSide();

      double yCenterDisplacementTowardsExpectedOtherFoot = (stanceSide == RobotSide.LEFT) ? -circleCenterOffset : circleCenterOffset;
      FrameVector3D translationFromFootCenterToCircleCenter = new FrameVector3D(stance.getSoleReferenceFrame(), 0.0, yCenterDisplacementTowardsExpectedOtherFoot,
            0.0);
      translationFromFootCenterToCircleCenter.changeFrame(ReferenceFrame.getWorldFrame());
      return translationFromFootCenterToCircleCenter;
   }

   public void setCheckForSwingThroughStance(boolean checkForSwingThroughStance)
   {
      this.checkForSwingThroughStance = checkForSwingThroughStance;
   }

}
