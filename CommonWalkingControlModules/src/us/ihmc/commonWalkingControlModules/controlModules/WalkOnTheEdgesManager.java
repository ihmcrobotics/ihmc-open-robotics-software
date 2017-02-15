package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class WalkOnTheEdgesManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean DO_TOEOFF_FOR_SIDE_STEPS = false;
   private static final boolean ENABLE_TOE_OFF_FOR_STEP_DOWN = true;

   private final BooleanYoVariable doToeOffIfPossible = new BooleanYoVariable("doToeOffIfPossible", registry);
   private final BooleanYoVariable doToeOffIfPossibleInSingleSupport = new BooleanYoVariable("doToeOffIfPossibleInSingleSupport", registry);
   private final BooleanYoVariable doToeOffWhenHittingAnkleLimit = new BooleanYoVariable("doToeOffWhenHittingAnkleLimit", registry);
   private final BooleanYoVariable doToeOff = new BooleanYoVariable("doToeOff", registry);

   private final DoubleYoVariable ankleLowerLimitToTriggerToeOff = new DoubleYoVariable("ankleLowerLimitToTriggerToeOff", registry);
   private final DoubleYoVariable icpProximityToLeadingFootForDSToeOff = new DoubleYoVariable("icpProximityToLeadingFootForDSToeOff", registry);
   private final DoubleYoVariable icpProximityToLeadingFootForSSToeOff = new DoubleYoVariable("icpProximityToLeadingFootForSSToeOff", registry);
   private final DoubleYoVariable icpPercentOfStanceForSSToeOff = new DoubleYoVariable("icpPercentOfStanceForSSToeOff", registry);

   private final BooleanYoVariable isDesiredICPOKForToeOff = new BooleanYoVariable("isDesiredICPOKForToeOff", registry);
   private final BooleanYoVariable isCurrentICPOKForToeOff = new BooleanYoVariable("isCurrentICPOKForToeOff", registry);
   private final BooleanYoVariable isDesiredECMPOKForToeOff = new BooleanYoVariable("isDesiredECMPOKForToeOff", registry);
   private final BooleanYoVariable needToSwitchToToeOffForAnkleLimit = new BooleanYoVariable("needToSwitchToToeOffForAnkleLimit", registry);

   private final DoubleYoVariable minStepLengthForToeOff = new DoubleYoVariable("minStepLengthForToeOff", registry);
   private final DoubleYoVariable minStepHeightForToeOff = new DoubleYoVariable("minStepHeightForToeOff", registry);

   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final List<FramePoint> contactStatePoints = new ArrayList<>();

   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final SideDependentList<FrameConvexPolygon2d> footDefaultPolygons;
   private final FrameConvexPolygon2d leadingFootSupportPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d onToesSupportPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d nextFootstepPolygon = new FrameConvexPolygon2d();

   private final DoubleYoVariable extraCoMMaxHeightWithToes = new DoubleYoVariable("extraCoMMaxHeightWithToes", registry);

   private final FramePoint tempLeadingFootPosition = new FramePoint();
   private final FramePoint tempTrailingFootPosition = new FramePoint();
   private final FramePoint tempLeadingFootPositionInWorld = new FramePoint();
   private final FramePoint tempTrailingFootPositionInWorld = new FramePoint();
   private final FrameVector toLeadingFoot = new FrameVector();

   private final WalkingControllerParameters walkingControllerParameters;

   private final BooleanYoVariable isRearAnklePitchHittingLimit;
   private final GlitchFilteredBooleanYoVariable isRearAnklePitchHittingLimitFilt;

   private final FullHumanoidRobotModel fullRobotModel;

   private final double inPlaceWidth;
   private final double footLength;

   public WalkOnTheEdgesManager(HighLevelHumanoidControllerToolbox momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         SideDependentList<? extends ContactablePlaneBody> feet, YoVariableRegistry parentRegistry)
   {
      this(momentumBasedController.getFullRobotModel(), walkingControllerParameters, feet, createFootContactStates(momentumBasedController), parentRegistry);
   }

   public WalkOnTheEdgesManager(FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters,
         SideDependentList<? extends ContactablePlaneBody> feet, SideDependentList<YoPlaneContactState> footContactStates,
         YoVariableRegistry parentRegistry)
   {
      this.doToeOffIfPossible.set(walkingControllerParameters.doToeOffIfPossible());
      this.doToeOffIfPossibleInSingleSupport.set(walkingControllerParameters.doToeOffIfPossibleInSingleSupport());
      this.doToeOffWhenHittingAnkleLimit.set(walkingControllerParameters.doToeOffWhenHittingAnkleLimit());

      this.ankleLowerLimitToTriggerToeOff.set(walkingControllerParameters.getAnkleLowerLimitToTriggerToeOff());
      this.icpProximityToLeadingFootForDSToeOff.set(walkingControllerParameters.getICPProximityToLeadingFootForToeOff());
      this.icpPercentOfStanceForSSToeOff.set(walkingControllerParameters.getICPPercentOfStanceForSSToeOff());

      this.walkingControllerParameters = walkingControllerParameters;

      this.fullRobotModel = fullRobotModel;
      this.feet = feet;

      this.inPlaceWidth = walkingControllerParameters.getInPlaceWidth();
      this.footLength = walkingControllerParameters.getFootBackwardOffset() + walkingControllerParameters.getFootForwardOffset();

      //TODO: extract param
      extraCoMMaxHeightWithToes.set(0.08);

      minStepLengthForToeOff.set(walkingControllerParameters.getMinStepLengthForToeOff());
      minStepHeightForToeOff.set(walkingControllerParameters.getMinStepHeightForToeOff());

      isRearAnklePitchHittingLimit = new BooleanYoVariable("isRearAnklePitchHittingLimit", registry);
      isRearAnklePitchHittingLimitFilt = new GlitchFilteredBooleanYoVariable("isRearAnklePitchHittingLimitFilt", registry, isRearAnklePitchHittingLimit, 10);

      footDefaultPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         footDefaultPolygons.put(robotSide, new FrameConvexPolygon2d(feet.get(robotSide).getContactPoints2d()));
      }

      this.footContactStates = footContactStates;

      parentRegistry.addChild(registry);
   }

   private static SideDependentList<YoPlaneContactState> createFootContactStates(HighLevelHumanoidControllerToolbox momentumBasedController)
   {
      SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         footContactStates.put(robotSide, momentumBasedController.getContactState(momentumBasedController.getContactableFeet().get(robotSide)));
      }
      return footContactStates;
   }

   /**
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in double support, and sets the {@link WalkOnTheEdgesManager#doToeOff} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    *   <li>doToeOffIfPossible</li>
    *   <li>desiredECMP location being within the support polygon account for toe-off, if {@link WalkingControllerParameters#checkECMPLocationToTriggerToeOff()} is true.</li>
    *   <li>desiredICP location being within the leading foot base of support.</li>
    *   <li>currentICP location being within the leading foot base of support.</li>
    *   <li>needToSwitchToToeOffForAnkleLimit</li>
    * </ol>
    * <p>
    * If able and the ankles are at the joint limits, transitions to toe-off. Then checks the current state being with the base of support. Then checks the
    * positioning of the leading leg to determine if it is acceptable.
    * </p>
    *
    * @param trailingLeg robot side for the trailing leg
    * @param desiredECMP current desired ECMP from ICP feedback.
    * @param desiredICP current desired ICP from the reference trajectory.
    * @param currentICP current ICP based on the robot state.
    */
   public void updateToeOffStatus(RobotSide trailingLeg, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      if (!doToeOffIfPossible.getBooleanValue())
      {
         doToeOff.set(false);
         isDesiredECMPOKForToeOff.set(false);
         return;
      }

      RobotSide leadingLeg = trailingLeg.getOppositeSide();
      if (footContactStates != null && footContactStates.get(leadingLeg).getTotalNumberOfContactPoints() > 0)
      {
         footContactStates.get(leadingLeg).getContactFramePointsInContact(contactStatePoints);
         leadingFootSupportPolygon.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(worldFrame, contactStatePoints);
      }
      else
      {
         leadingFootSupportPolygon.setIncludingFrameAndUpdate(footDefaultPolygons.get(leadingLeg));
         leadingFootSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      }

      updateOnToesSupportPolygon(trailingLeg, leadingFootSupportPolygon);

      if (walkingControllerParameters.checkECMPLocationToTriggerToeOff())
         isDesiredECMPOKForToeOff.set(onToesSupportPolygon.isPointInside(desiredECMP));
      else
         isDesiredECMPOKForToeOff.set(true);

      boolean isDesiredICPOKForToeOff, isCurrentICPOKForToeOff;
      if (icpProximityToLeadingFootForDSToeOff.getDoubleValue() > 0.0)
      {
         isDesiredICPOKForToeOff =
               onToesSupportPolygon.isPointInside(desiredICP) && leadingFootSupportPolygon.distance(desiredICP) < (icpProximityToLeadingFootForDSToeOff.getDoubleValue());
         isCurrentICPOKForToeOff =
               onToesSupportPolygon.isPointInside(currentICP) && leadingFootSupportPolygon.distance(currentICP) < (icpProximityToLeadingFootForDSToeOff.getDoubleValue());
      }
      else
      {
         isDesiredICPOKForToeOff = leadingFootSupportPolygon.isPointInside(desiredICP);
         isCurrentICPOKForToeOff = leadingFootSupportPolygon.isPointInside(currentICP);
      }

      this.isDesiredICPOKForToeOff.set(isDesiredICPOKForToeOff);
      this.isCurrentICPOKForToeOff.set(isCurrentICPOKForToeOff);

      boolean finishedChecks = checkToeOffConditions(trailingLeg);
      if (finishedChecks)
         return;

      isReadyToSwitchToToeOff(trailingLeg, feet.get(leadingLeg).getFrameAfterParentJoint());
   }

   /**
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in single support, and sets the {@link WalkOnTheEdgesManager#doToeOff} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    *   <li>doToeOffIfPossibleInSingleSupport</li>
    *   <li>needToSwitchToToeOffForAnkleLimit</li>
    *   <li>isOnExitCMP</li>
    * </ol>
    * <p>
    * If single support toe-off is enabled, the ankle is at its indicated limit, and the desired ECMP is on the exit ECMP,
    * transitions to toe-off. Then checks the position of the leading leg to determine if it is acceptable.
    * </p>
    *
    * @param nextFootstep the upcoming footstep
    * @param desiredECMP current location of the desired ECMP
    * @param currentICP current location of the instantaneous capture point
    * @param desiredICP desired location of the instantaneous capture point
    * @param isOnExitCMP boolean as to whether or not the current ICP plan is attempting to use the exit CMP. Sets the variable {@link WalkOnTheEdgesManager#isDesiredECMPOKForToeOff}.
    */
   public void updateToeOffStatusSingleSupport(Footstep nextFootstep, FramePoint2d desiredECMP, FramePoint2d currentICP, FramePoint2d desiredICP, boolean isOnExitCMP)
   {
      if (!doToeOffIfPossibleInSingleSupport.getBooleanValue())
      {
         doToeOff.set(false);
         isDesiredECMPOKForToeOff.set(false);
         return;
      }

      ReferenceFrame footstepSoleFrame = nextFootstep.getSoleReferenceFrame();
      List<Point2d> predictedContactPoints = nextFootstep.getPredictedContactPoints();
      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         nextFootstepPolygon.setIncludingFrameAndUpdate(footstepSoleFrame, predictedContactPoints);
      }
      else
      {
         ConvexPolygon2d footPolygon = footDefaultPolygons.get(nextFootstep.getRobotSide()).getConvexPolygon2d();
         nextFootstepPolygon.setIncludingFrameAndUpdate(footstepSoleFrame, footPolygon);
      }

      RobotSide trailingLeg = nextFootstep.getRobotSide().getOppositeSide();
      nextFootstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      updateOnToesSupportPolygon(trailingLeg, nextFootstepPolygon);

      if (walkingControllerParameters.checkECMPLocationToTriggerToeOff())
         isDesiredECMPOKForToeOff.set(isOnExitCMP && onToesSupportPolygon.isPointInside(desiredECMP));
      else
         isDesiredECMPOKForToeOff.set(isOnExitCMP);

      boolean isDesiredICPOKForToeOff, isCurrentICPOKForToeOff;
      if (icpPercentOfStanceForSSToeOff.getDoubleValue() > 0.0)
      {
         // compute stance length
         ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getFrameAfterParentJoint();
         tempLeadingFootPosition.setToZero(footstepSoleFrame);
         tempTrailingFootPosition.setToZero(trailingFootFrame);
         tempLeadingFootPosition.changeFrame(trailingFootFrame);

         toLeadingFoot.setToZero(trailingFootFrame);
         toLeadingFoot.set(tempLeadingFootPosition);
         toLeadingFoot.sub(tempTrailingFootPosition);

         icpProximityToLeadingFootForSSToeOff.set(icpPercentOfStanceForSSToeOff.getDoubleValue() * toLeadingFoot.length());
         isDesiredICPOKForToeOff =
               onToesSupportPolygon.isPointInside(desiredICP) && nextFootstepPolygon.distance(desiredICP) < (icpProximityToLeadingFootForSSToeOff.getDoubleValue());
         isCurrentICPOKForToeOff =
               onToesSupportPolygon.isPointInside(currentICP) && nextFootstepPolygon.distance(currentICP) < (icpProximityToLeadingFootForSSToeOff.getDoubleValue());
      }
      else
      {
         isDesiredICPOKForToeOff = nextFootstepPolygon.isPointInside(desiredICP);
         isCurrentICPOKForToeOff = nextFootstepPolygon.isPointInside(currentICP);
      }

      this.isCurrentICPOKForToeOff.set(isCurrentICPOKForToeOff);
      this.isDesiredICPOKForToeOff.set(isDesiredICPOKForToeOff);

      boolean finishedChecks = checkToeOffConditions(trailingLeg);
      if (finishedChecks)
         return;

      isReadyToSwitchToToeOff(trailingLeg, footstepSoleFrame);
   }

   private boolean checkToeOffConditions(RobotSide trailingLeg)
   {
      if (!this.isDesiredICPOKForToeOff.getBooleanValue() || !this.isCurrentICPOKForToeOff.getBooleanValue())
      {
         doToeOff.set(false);
         return true;
      }

      needToSwitchToToeOffForAnkleLimit.set(checkAnkleLimitForToeOff(trailingLeg));
      if (needToSwitchToToeOffForAnkleLimit.getBooleanValue())
      {
         doToeOff.set(true);
         return true;
      }

      if (!isDesiredECMPOKForToeOff.getBooleanValue())
      {
         doToeOff.set(false);
         return true;
      }

      return false;
   }

   private boolean checkAnkleLimitForToeOff(RobotSide trailingLeg)
   {
      OneDoFJoint anklePitch = fullRobotModel.getLegJoint(trailingLeg, LegJointName.ANKLE_PITCH);
      double lowerLimit = Math.max(anklePitch.getJointLimitLower() + 0.02, ankleLowerLimitToTriggerToeOff.getDoubleValue());
      isRearAnklePitchHittingLimit.set(anklePitch.getQ() < lowerLimit);
      isRearAnklePitchHittingLimitFilt.update();

      if (!doToeOffWhenHittingAnkleLimit.getBooleanValue())
         return false;

      if (!isDesiredICPOKForToeOff.getBooleanValue() || !isCurrentICPOKForToeOff.getBooleanValue())
         return false;

      return isRearAnklePitchHittingLimitFilt.getBooleanValue();
   }

   private void isReadyToSwitchToToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      if (!isFrontFootWellPositionedForToeOff(trailingLeg, frontFootFrame))
      {
         doToeOff.set(false);
         return;
      }

      doToeOff.set(true);
   }

   private boolean isFrontFootWellPositionedForToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getFrameAfterParentJoint();
      tempLeadingFootPosition.setToZero(frontFootFrame);
      tempTrailingFootPosition.setToZero(trailingFootFrame);
      tempLeadingFootPosition.changeFrame(trailingFootFrame);

      if (Math.abs(tempLeadingFootPosition.getY()) > inPlaceWidth)
         tempLeadingFootPosition.setY(tempLeadingFootPosition.getY() + trailingLeg.negateIfRightSide(inPlaceWidth));
      else
         tempLeadingFootPosition.setY(0.0);

      tempLeadingFootPositionInWorld.setToZero(frontFootFrame);
      tempTrailingFootPositionInWorld.setToZero(trailingFootFrame);
      tempLeadingFootPositionInWorld.changeFrame(worldFrame);
      tempTrailingFootPositionInWorld.changeFrame(worldFrame);

      double stepHeight = tempLeadingFootPositionInWorld.getZ() - tempTrailingFootPositionInWorld.getZ();

      boolean isNextStepHighEnough = stepHeight > minStepHeightForToeOff.getDoubleValue();
      if (isNextStepHighEnough)
         return true;

      boolean isForwardOrSideStepping = tempLeadingFootPosition.getX() > -0.05;
      if (!isForwardOrSideStepping)
         return false;

      if (ENABLE_TOE_OFF_FOR_STEP_DOWN)
      {
         boolean isNextStepLowEnough = stepHeight < -minStepHeightForToeOff.getDoubleValue();
         if (isNextStepLowEnough)
            return true;
      }
      else
      {
         boolean isNextStepTooLow = stepHeight < -0.10;
         if (isNextStepTooLow)
            return false;
      }

      boolean isSideStepping = Math.abs(Math.atan2(tempLeadingFootPosition.getY(), tempLeadingFootPosition.getX())) > Math.toRadians(45.0);
      if (isSideStepping && !DO_TOEOFF_FOR_SIDE_STEPS)
         return false;

      boolean isStepLongEnough = tempLeadingFootPosition.distance(tempTrailingFootPosition) > minStepLengthForToeOff.getDoubleValue();
      boolean isStepLongEnoughAlongX = tempLeadingFootPosition.getX() > footLength;
      return isStepLongEnough && isStepLongEnoughAlongX;
   }

   public boolean willDoToeOff(Footstep nextFootstep, RobotSide transferToSide)
   {
      if (!doToeOffIfPossible.getBooleanValue())
         return false;

      RobotSide nextTrailingLeg = transferToSide.getOppositeSide();
      ReferenceFrame nextFrontFootFrame;
      if (nextFootstep != null)
         nextFrontFootFrame = nextFootstep.getPoseReferenceFrame();
      else
         nextFrontFootFrame = feet.get(nextTrailingLeg.getOppositeSide()).getFrameAfterParentJoint();

      boolean frontFootWellPositionedForToeOff = isFrontFootWellPositionedForToeOff(nextTrailingLeg, nextFrontFootFrame);

      return frontFootWellPositionedForToeOff;
   }

   public boolean doToeOff()
   {
      return doToeOff.getBooleanValue();
   }

   public boolean doToeOffIfPossible()
   {
      return doToeOffIfPossible.getBooleanValue();
   }

   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return doToeOffIfPossibleInSingleSupport.getBooleanValue();
   }

   public void setDoToeOffIfPossible(boolean value)
   {
      doToeOffIfPossible.set(value);
   }

   public double getExtraCoMMaxHeightWithToes()
   {
      return extraCoMMaxHeightWithToes.getDoubleValue();
   }

   public void reset()
   {
      isDesiredECMPOKForToeOff.set(false);
      isDesiredICPOKForToeOff.set(false);

      isRearAnklePitchHittingLimit.set(false);
      isRearAnklePitchHittingLimitFilt.set(false);

      doToeOff.set(false);
   }

   private final FramePoint[] toePoints = new FramePoint[] {new FramePoint(), new FramePoint()};
   private final FramePoint middleToePoint = new FramePoint();
   private final FramePoint2d footPoint = new FramePoint2d();

   private void computeToePoints(RobotSide supportSide)
   {
      FrameConvexPolygon2d footDefaultPolygon = footDefaultPolygons.get(supportSide);
      toePoints[0].setIncludingFrame(footDefaultPolygon.getReferenceFrame(), Double.NEGATIVE_INFINITY, 0.0, 0.0);
      toePoints[1].setIncludingFrame(footDefaultPolygon.getReferenceFrame(), Double.NEGATIVE_INFINITY, 0.0, 0.0);

      for (int i = 0; i < footDefaultPolygon.getNumberOfVertices(); i++)
      {
         footDefaultPolygon.getFrameVertex(i, footPoint);
         if (footPoint.getX() > toePoints[0].getX())
         {
            toePoints[1].set(toePoints[0]);
            toePoints[0].setXY(footPoint);
         }
         else if (footPoint.getX() > toePoints[1].getX())
         {
            toePoints[1].setXY(footPoint);
         }
      }

      middleToePoint.setToZero(footDefaultPolygon.getReferenceFrame());
      middleToePoint.interpolate(toePoints[0], toePoints[1], 0.5);
   }


   private void updateOnToesSupportPolygon(RobotSide trailingSide, FrameConvexPolygon2d leadingFootSupportPolygon)
   {
      computeToePoints(trailingSide);
      middleToePoint.changeFrame(worldFrame);

      onToesSupportPolygon.setIncludingFrameAndUpdate(leadingFootSupportPolygon);
      onToesSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      onToesSupportPolygon.addVertexByProjectionOntoXYPlane(middleToePoint);
      onToesSupportPolygon.update();
   }
}