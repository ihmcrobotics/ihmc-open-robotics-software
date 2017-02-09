package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import static org.junit.Assert.assertTrue;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepValidityMetric;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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

   public boolean assertValid(Footstep swingStart, Footstep stance, Footstep swingEnd)
   {
      return checkIfValid(swingStart, stance, swingEnd, true);
   }

   public boolean assertValid(String message, Footstep swingStart, Footstep stance, Footstep swingEnd)
   {
      return checkIfValid(message, swingStart, stance, swingEnd, true);
   }

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
      FramePose stancePose = new FramePose();
      stance.getSolePose(stancePose);
      FramePose swingEndPose = new FramePose();
      swingEnd.getSolePose(swingEndPose);
      FrameOrientation2d stanceOrientation = new FrameOrientation2d();
      FrameOrientation2d swingOrientation = new FrameOrientation2d();
      stancePose.getOrientation2dIncludingFrame(stanceOrientation);
      swingEndPose.getOrientation2dIncludingFrame(swingOrientation);
      double yawDifference = swingOrientation.sub(stanceOrientation);
      if (Math.abs(yawDifference) > footTwistLimitInRadians + 1e-15)
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
      FrameVector translationFromFootCenterToCircleCenter = getTranslationVector(stance, circleCenterOffset);
      FrameVector vectorToSwingEnd = offCenterVectorToSwingEnd(stance, swingEnd, translationFromFootCenterToCircleCenter);
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

      FramePose initialFramePose = new FramePose();
      FramePose stanceFramePose = new FramePose();
      FramePose endingFramePose = new FramePose();
      swingStart.getSolePose(initialFramePose);
      stance.getSolePose(stanceFramePose);
      swingEnd.getSolePose(endingFramePose);

      FramePoint2d initialPoint = new FramePoint2d();
      initialFramePose.getPosition2dIncludingFrame(initialPoint);
      FramePoint2d stancePoint = new FramePoint2d();
      stanceFramePose.getPosition2dIncludingFrame(stancePoint);
      FramePoint2d endPoint = new FramePoint2d();
      endingFramePose.getPosition2dIncludingFrame(endPoint);
      boolean swingsThroughStanceLeg;
      if ((initialPoint.getX() == endPoint.getX()) && (initialPoint.getY() == endPoint.getY()))
      {
         swingsThroughStanceLeg = false;
      }
      else
      {
         FrameLineSegment2d swingLine = new FrameLineSegment2d(initialPoint, endPoint);
         double swingLineDistance = swingLine.distance(stancePoint);

         swingsThroughStanceLeg = checkIfSwingGoesThroughStanceLeg(stance, stanceFramePose, swingLine);

         swingsThroughStanceLeg |= swingLineDistance < circleCenterOffset;
      }

      valid = priorValidity && (valid1 || valid2) && !swingsThroughStanceLeg;
      assertValidIfTrue(message + " Check for footstep side acceptability and no swing through stance leg", assertValidity);
   }

   private boolean checkIfSwingGoesThroughStanceLeg(Footstep stance, FramePose stanceFramePose, FrameLineSegment2d swingLine)
   {
      RobotSide stanceFootstepSide = stance.getRobotSide();

      FrameVector offset1 = getPerpendicularOffset(stanceFootstepSide, stanceFramePose, circleCenterOffset);
      FrameVector offset2 = getPerpendicularOffset(stanceFootstepSide, stanceFramePose, -3 * footReachLimitInMeters);
      FramePoint2d stanceInsideOffset = new FramePoint2d(ReferenceFrame.getWorldFrame(), offset1.getX(), offset1.getY());
      FramePoint2d stanceOutsideOffset = new FramePoint2d(ReferenceFrame.getWorldFrame(), offset2.getX(), offset2.getY());
      FrameLineSegment2d crossThroughLine = new FrameLineSegment2d(stanceInsideOffset, stanceOutsideOffset);
      FramePoint2d intersectionPoint = crossThroughLine.intersectionWith(swingLine);
      boolean swingsThroughStanceLeg;
      if (intersectionPoint == null)
         swingsThroughStanceLeg = false;
      else
         swingsThroughStanceLeg = true;
      return swingsThroughStanceLeg;
   }

   private static FrameVector getPerpendicularOffset(RobotSide stanceFootstepSide, FramePose stanceFramePose, double semiCircleOffset)
   {
      double sideOffset = semiCircleOffset * ((stanceFootstepSide == RobotSide.LEFT) ? -1 : 1);
      double offsetDirAngle = stanceFramePose.getYaw() + Math.PI / 2;
      FrameVector offset = new FrameVector(ReferenceFrame.getWorldFrame(), new double[] { Math.cos(offsetDirAngle) * sideOffset,
            Math.sin(offsetDirAngle) * sideOffset, 0 });

      FramePoint stanceFramePosition = new FramePoint();
      stanceFramePose.getPositionIncludingFrame(stanceFramePosition);
      FrameVector offsetStart = new FrameVector(stanceFramePosition);
      offsetStart.add(offset);

      return offsetStart;
   }

   private FrameVector offCenterVectorToSwingEnd(Footstep stance, Footstep swingEnd, FrameVector translationFromFootCenterToCircleCenter)
   {
      ReferenceFrame stanceFootFrame = stance.getParentFrame();

      FramePoint circleCenter;
      FramePose stanceSole = new FramePose();
      FramePose swingEndSole = new FramePose();
      stance.getSolePose(stanceSole);
      swingEnd.getSolePose(swingEndSole);

      circleCenter = stanceSole.getFramePointCopy();
      circleCenter.add(translationFromFootCenterToCircleCenter);//change frame of translation first?

      FrameVector vectorToSwingEnd = new FrameVector(stanceFootFrame);
      vectorToSwingEnd.set(swingEndSole.getFramePointCopy().getPoint());
      vectorToSwingEnd.sub(circleCenter);

      return vectorToSwingEnd;
   }

   private FrameVector getTranslationVector(Footstep stance, double circleCenterOffset)
   {
      RobotSide stanceSide = stance.getRobotSide();

      double yCenterDisplacementTowardsExpectedOtherFoot = (stanceSide == RobotSide.LEFT) ? -circleCenterOffset : circleCenterOffset;
      FrameVector translationFromFootCenterToCircleCenter = new FrameVector(stance.getSoleReferenceFrame(), 0.0, yCenterDisplacementTowardsExpectedOtherFoot,
            0.0);
      translationFromFootCenterToCircleCenter.changeFrame(stance.getParentFrame());
      return translationFromFootCenterToCircleCenter;
   }

   public void setCheckForSwingThroughStance(boolean checkForSwingThroughStance)
   {
      this.checkForSwingThroughStance = checkForSwingThroughStance;
   }

}
