package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
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

   private static final int largeGlitchWindowSize = 10;
   private static final int smallGlitchWindowSize = 2;

   private final BooleanYoVariable doToeOffIfPossibleInDoubleSupport = new BooleanYoVariable("doToeOffIfPossibleInDoubleSupport", registry);
   private final BooleanYoVariable doToeOffIfPossibleInSingleSupport = new BooleanYoVariable("doToeOffIfPossibleInSingleSupport", registry);
   private final BooleanYoVariable doToeOffWhenHittingAnkleLimit = new BooleanYoVariable("doToeOffWhenHittingAnkleLimit", registry);
   private final BooleanYoVariable doPointToeOff = new BooleanYoVariable("doPointToeOff", registry);
   private final BooleanYoVariable doLineToeOff = new BooleanYoVariable("doLineToeOff", registry);

   private final BooleanYoVariable useToeLineContactInSwing = new BooleanYoVariable("useToeLineContactInSwing", registry);
   private final BooleanYoVariable useToeLineContactInTransfer = new BooleanYoVariable("useToeLineContactInTransfer", registry);
   private final BooleanYoVariable computeToeLineContact = new BooleanYoVariable("computeToeLineContact", registry);
   private final BooleanYoVariable computeToePointContact = new BooleanYoVariable("computeToePointContact", registry);
   private final BooleanYoVariable updateLineContactDuringToeOff = new BooleanYoVariable("updateLineContactDuringToeOff", registry);
   private final BooleanYoVariable updatePointContactDuringToeOff = new BooleanYoVariable("updatePointContactDuringToeOff", registry);

   private final DoubleYoVariable ankleLowerLimitToTriggerToeOff = new DoubleYoVariable("ankleLowerLimitToTriggerToeOff", registry);
   private final DoubleYoVariable icpProximityToLeadingFootForDSToeOff = new DoubleYoVariable("icpProximityToLeadingFootForDSToeOff", registry);
   private final DoubleYoVariable icpProximityToLeadingFootForSSToeOff = new DoubleYoVariable("icpProximityToLeadingFootForSSToeOff", registry);
   private final DoubleYoVariable icpPercentOfStanceForDSToeOff = new DoubleYoVariable("icpPercentOfStanceForDSToeOff", registry);
   private final DoubleYoVariable icpPercentOfStanceForSSToeOff = new DoubleYoVariable("icpPercentOfStanceForSSToeOff", registry);
   private final DoubleYoVariable ecmpProximityForToeOff = new DoubleYoVariable("ecmpProximityForToeOff", registry);

   private final BooleanYoVariable isDesiredICPOKForToeOff = new BooleanYoVariable("isDesiredICPOKForToeOff", registry);
   private final BooleanYoVariable isCurrentICPOKForToeOff = new BooleanYoVariable("isCurrentICPOKForToeOff", registry);
   private final BooleanYoVariable isDesiredECMPOKForToeOff = new BooleanYoVariable("isDesiredECMPOKForToeOff", registry);
   private final BooleanYoVariable needToSwitchToToeOffForAnkleLimit = new BooleanYoVariable("needToSwitchToToeOffForAnkleLimit", registry);
   private final BooleanYoVariable isRearAnklePitchHittingLimit = new BooleanYoVariable("isRearAnklePitchHittingLimit", registry);

   private final GlitchFilteredBooleanYoVariable isDesiredICPOKForToeOffFilt = new GlitchFilteredBooleanYoVariable("isDesiredICPOKForToeOffFilt",
         registry, isDesiredICPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredBooleanYoVariable isCurrentICPOKForToeOffFilt = new GlitchFilteredBooleanYoVariable("isCurrentICPOKForToeOffFilt",
         registry, isCurrentICPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredBooleanYoVariable isDesiredECMPOKForToeOffFilt = new GlitchFilteredBooleanYoVariable("isDesiredECMPOKForToeOffFilt",
         registry, isDesiredECMPOKForToeOff, smallGlitchWindowSize);

   private final GlitchFilteredBooleanYoVariable isRearAnklePitchHittingLimitFilt = new GlitchFilteredBooleanYoVariable("isRearAnklePitchHittingLimitFilt",
         registry, isRearAnklePitchHittingLimit, largeGlitchWindowSize);

   private final BooleanYoVariable isInSingleSupport = new BooleanYoVariable("isInSingleSupport", registry);

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

   private final FramePoint2d tmpPoint2d = new FramePoint2d();
   private final FramePoint2d toeOffPoint = new FramePoint2d();
   private final FrameLineSegment2d toeOffLine = new FrameLineSegment2d();

   private Footstep nextFootstep;

   private final WalkingControllerParameters walkingControllerParameters;

   private final FullHumanoidRobotModel fullRobotModel;
   private final ToeOffCalculator toeOffCalculator;

   private final double inPlaceWidth;
   private final double footLength;

   public WalkOnTheEdgesManager(HighLevelHumanoidControllerToolbox controllerToolbox, ToeOffCalculator toeOffCalculator,
         WalkingControllerParameters walkingControllerParameters, SideDependentList<? extends ContactablePlaneBody> feet, YoVariableRegistry parentRegistry)
   {
      this(controllerToolbox.getFullRobotModel(), toeOffCalculator, walkingControllerParameters, feet, createFootContactStates(controllerToolbox), parentRegistry);
   }

   public WalkOnTheEdgesManager(FullHumanoidRobotModel fullRobotModel, ToeOffCalculator toeOffCalculator,
         WalkingControllerParameters walkingControllerParameters, SideDependentList<? extends ContactablePlaneBody> feet,
         SideDependentList<YoPlaneContactState> footContactStates, YoVariableRegistry parentRegistry)
   {
      this.doToeOffIfPossibleInDoubleSupport.set(walkingControllerParameters.doToeOffIfPossible());
      this.doToeOffIfPossibleInSingleSupport.set(walkingControllerParameters.doToeOffIfPossibleInSingleSupport());
      this.doToeOffWhenHittingAnkleLimit.set(walkingControllerParameters.doToeOffWhenHittingAnkleLimit());

      this.ankleLowerLimitToTriggerToeOff.set(walkingControllerParameters.getAnkleLowerLimitToTriggerToeOff());
      this.icpPercentOfStanceForDSToeOff.set(walkingControllerParameters.getICPPercentOfStanceForDSToeOff());
      this.icpPercentOfStanceForSSToeOff.set(walkingControllerParameters.getICPPercentOfStanceForSSToeOff());

      this.ecmpProximityForToeOff.set(walkingControllerParameters.getECMPProximityForToeOff());

      this.toeOffCalculator = toeOffCalculator;
      this.walkingControllerParameters = walkingControllerParameters;

      this.fullRobotModel = fullRobotModel;
      this.feet = feet;

      this.inPlaceWidth = walkingControllerParameters.getInPlaceWidth();
      this.footLength = walkingControllerParameters.getFootBackwardOffset() + walkingControllerParameters.getFootForwardOffset();

      //TODO: extract param
      extraCoMMaxHeightWithToes.set(0.08);

      minStepLengthForToeOff.set(walkingControllerParameters.getMinStepLengthForToeOff());
      minStepHeightForToeOff.set(walkingControllerParameters.getMinStepHeightForToeOff());

      useToeLineContactInSwing.set(true);
      useToeLineContactInTransfer.set(false);

      footDefaultPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         footDefaultPolygons.put(robotSide, new FrameConvexPolygon2d(feet.get(robotSide).getContactPoints2d()));
      }

      this.footContactStates = footContactStates;

      parentRegistry.addChild(registry);
   }

   private static SideDependentList<YoPlaneContactState> createFootContactStates(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         footContactStates.put(robotSide, controllerToolbox.getFootContactState(robotSide));
      }
      return footContactStates;
   }

   /**
    * Sets the upcoming footstep, which is used to predict the support polygon in single support.
    * @param nextFootstep
    */
   public void submitNextFootstep(Footstep nextFootstep)
   {
      this.nextFootstep = nextFootstep;
   }

   /**
    * Tells the internal methods to execute as if in single support.
    */
   public void inSingleSupport()
   {
      isInSingleSupport.set(true);
   }

   /**
    * Tells the internal methods to execute as if in double support.
    */
   public void inDoubleSupport()
   {
      nextFootstep = null;
      isInSingleSupport.set(false);
   }

   public boolean updateToeOffStatusSingleSupport(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      if (!doToeOffIfPossibleInSingleSupport.getBooleanValue())
      {
         doPointToeOff.set(false);
         isDesiredECMPOKForToeOff.set(false);
         isDesiredECMPOKForToeOffFilt.set(false);
         return false;
      }

      boolean doToeOff;
      if (useToeLineContactInSwing.getBooleanValue())
      {
         updateLineToeOffStatus(trailingLeg, exitCMP, desiredECMP, desiredICP, currentICP);
         doToeOff = doLineToeOff.getBooleanValue();
      }
      else
      {
         updatePointToeOffStatus(trailingLeg, exitCMP, desiredECMP, desiredICP, currentICP);
         doToeOff = doPointToeOff.getBooleanValue();
      }
      return doToeOff;
   }

   public boolean updateToeOffStatusDoubleSupport(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      boolean doToeOff;
      if (useToeLineContactInTransfer.getBooleanValue())
      {
         updateLineToeOffStatus(trailingLeg, exitCMP, desiredECMP, desiredICP, currentICP);
         doToeOff = doLineToeOff.getBooleanValue();
      }
      else
      {
         updatePointToeOffStatus(trailingLeg, exitCMP, desiredECMP, desiredICP, currentICP);
         doToeOff = doPointToeOff.getBooleanValue();
      }

      return doToeOff;
   }

   /**
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in double support, and sets the {@link WalkOnTheEdgesManager#doLineToeOff} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    *   <li>doToeOffIfPossibleInDoubleSupport</li>
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
   private void updateLineToeOffStatus(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      ReferenceFrame soleFrame;
      if (isInSingleSupport.getBooleanValue())
      {
         if (!doToeOffIfPossibleInSingleSupport.getBooleanValue())
         {
            doLineToeOff.set(false);
            isDesiredECMPOKForToeOff.set(false);

            isDesiredECMPOKForToeOffFilt.set(false);
            computeToeLineContact.set(false);
            return;
         }
         else
         {
            updateLineToeOffStatusSingleSupport(exitCMP, desiredECMP);
            soleFrame = nextFootstep.getSoleReferenceFrame();
            checkICPLocations(trailingLeg, desiredICP, currentICP, nextFootstepPolygon);
         }
      }
      else
      {
         if (!doToeOffIfPossibleInSingleSupport.getBooleanValue())
         {
            doLineToeOff.set(false);
            isDesiredECMPOKForToeOff.set(false);
            needToSwitchToToeOffForAnkleLimit.set(false);

            isDesiredECMPOKForToeOffFilt.set(false);
            computeToeLineContact.set(false);
            return;
         }
         else
         {
            updateLineToeOffStatusDoubleSupport(trailingLeg, exitCMP, desiredECMP);
            soleFrame = feet.get(trailingLeg.getOppositeSide()).getFrameAfterParentJoint();
            checkICPLocations(trailingLeg, desiredICP, currentICP, leadingFootSupportPolygon);
         }
      }

      checkECMPLocation(desiredECMP);

      if (!evaluateLineToeOffConditions(trailingLeg))
         return;

      isReadyToSwitchToLineToeOff(trailingLeg, soleFrame);
   }

   /**
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in doubee support, and sets the {@link WalkOnTheEdgesManager#doPointToeOff} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    *   <li>doToeOffIfPossibleInDoubleSupport</li>
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
   private void updatePointToeOffStatus(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      ReferenceFrame soleFrame;
      if (isInSingleSupport.getBooleanValue())
      {
         if (!doToeOffIfPossibleInSingleSupport.getBooleanValue())
         {
            doPointToeOff.set(false);
            isDesiredECMPOKForToeOff.set(false);
            isDesiredECMPOKForToeOffFilt.set(false);
            computeToePointContact.set(false);
            return;
         }
         else
         {
            updatePointToeOffStatusSingleSupport(exitCMP, desiredECMP);
            soleFrame = nextFootstep.getSoleReferenceFrame();
            checkICPLocations(trailingLeg, desiredICP, currentICP, nextFootstepPolygon);
         }
      }
      else
      {
         if (!doToeOffIfPossibleInDoubleSupport.getBooleanValue())
         {
            doPointToeOff.set(false);
            isDesiredECMPOKForToeOff.set(false);
            needToSwitchToToeOffForAnkleLimit.set(false);

            isDesiredECMPOKForToeOffFilt.set(false);
            computeToePointContact.set(false);
            return;
         }
         else
         {
            updatePointToeOffStatusDoubleSupport(trailingLeg, exitCMP, desiredECMP);
            soleFrame = feet.get(trailingLeg.getOppositeSide()).getFrameAfterParentJoint();
            checkICPLocations(trailingLeg, desiredICP, currentICP, leadingFootSupportPolygon);
         }
      }

      checkECMPLocation(desiredECMP);

      if (!evaluatePointToeOffConditions(trailingLeg))
         return;

      isReadyToSwitchToPointToeOff(trailingLeg, soleFrame);
   }

   private void updateLineToeOffStatusDoubleSupport(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredECMP)
   {
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

      updateOnLineToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, leadingFootSupportPolygon);
   }

   private void updatePointToeOffStatusDoubleSupport(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredECMP)
   {
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

      updateOnPointToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, leadingFootSupportPolygon);
   }

   private void updateLineToeOffStatusSingleSupport(FramePoint exitCMP, FramePoint2d desiredECMP)
   {
      setNextFootstepPolygon();

      RobotSide trailingLeg = nextFootstep.getRobotSide().getOppositeSide();
      nextFootstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      updateOnLineToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, nextFootstepPolygon);
   }

   private void updatePointToeOffStatusSingleSupport(FramePoint exitCMP, FramePoint2d desiredECMP)
   {
      setNextFootstepPolygon();

      RobotSide trailingLeg = nextFootstep.getRobotSide().getOppositeSide();
      nextFootstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      updateOnPointToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, nextFootstepPolygon);
   }

   private void setNextFootstepPolygon()
   {
      if (nextFootstep == null)
         throw new RuntimeException("The next footstep has not been set.");

      ReferenceFrame footstepSoleFrame = nextFootstep.getSoleReferenceFrame();
      List<Point2D> predictedContactPoints = nextFootstep.getPredictedContactPoints();
      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         nextFootstepPolygon.setIncludingFrameAndUpdate(footstepSoleFrame, predictedContactPoints);
      }
      else
      {
         ConvexPolygon2d footPolygon = footDefaultPolygons.get(nextFootstep.getRobotSide()).getConvexPolygon2d();
         nextFootstepPolygon.setIncludingFrameAndUpdate(footstepSoleFrame, footPolygon);
      }
   }

   private void checkECMPLocation(FramePoint2d desiredECMP)
   {
      if (walkingControllerParameters.checkECMPLocationToTriggerToeOff())
      {
         desiredECMP.changeFrameAndProjectToXYPlane(onToesSupportPolygon.getReferenceFrame());
         isDesiredECMPOKForToeOff.set(onToesSupportPolygon.distance(desiredECMP) <= ecmpProximityForToeOff.getDoubleValue());
         isDesiredECMPOKForToeOffFilt.update();
      }
      else
      {
         isDesiredECMPOKForToeOff.set(true);
         isDesiredECMPOKForToeOffFilt.update();
      }
   }

   private void checkICPLocations(RobotSide trailingLeg, FramePoint2d desiredICP, FramePoint2d currentICP, FrameConvexPolygon2d leadingFootSupportPolygon)
   {
      double proximityState;
      if (isInSingleSupport.getBooleanValue())
         proximityState = icpPercentOfStanceForSSToeOff.getDoubleValue();
      else
         proximityState = icpPercentOfStanceForDSToeOff.getDoubleValue();

      boolean isDesiredICPOKForToeOff, isCurrentICPOKForToeOff;
      if (proximityState > 0.0)
      {
         // compute stance length
         double requiredProximity = computeRequiredICPProximity(trailingLeg);

         isDesiredICPOKForToeOff =
               onToesSupportPolygon.isPointInside(desiredICP) && leadingFootSupportPolygon.distance(desiredICP) < requiredProximity;
         isCurrentICPOKForToeOff =
               onToesSupportPolygon.isPointInside(currentICP) && leadingFootSupportPolygon.distance(currentICP) < requiredProximity;
      }
      else
      {
         isDesiredICPOKForToeOff = leadingFootSupportPolygon.isPointInside(desiredICP);
         isCurrentICPOKForToeOff = leadingFootSupportPolygon.isPointInside(currentICP);
      }

      this.isCurrentICPOKForToeOff.set(isCurrentICPOKForToeOff);
      this.isDesiredICPOKForToeOff.set(isDesiredICPOKForToeOff);
      this.isCurrentICPOKForToeOffFilt.update();
      this.isDesiredICPOKForToeOffFilt.update();
   }

   private double computeRequiredICPProximity(RobotSide trailingLeg)
   {
      if (isInSingleSupport.getBooleanValue())
      {
         ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getFrameAfterParentJoint();
         ReferenceFrame footstepSoleFrame = nextFootstep.getSoleReferenceFrame();
         tempLeadingFootPosition.setToZero(footstepSoleFrame);
         tempTrailingFootPosition.setToZero(trailingFootFrame);
         tempLeadingFootPosition.changeFrame(trailingFootFrame);

         toLeadingFoot.setToZero(trailingFootFrame);
         toLeadingFoot.set(tempLeadingFootPosition);
         toLeadingFoot.sub(tempTrailingFootPosition);

         icpProximityToLeadingFootForSSToeOff.set(icpPercentOfStanceForSSToeOff.getDoubleValue() * toLeadingFoot.length());

         return icpProximityToLeadingFootForSSToeOff.getDoubleValue();
      }
      else
      {
         ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getFrameAfterParentJoint();
         ReferenceFrame footstepSoleFrame = feet.get(trailingLeg.getOppositeSide()).getFrameAfterParentJoint();
         tempLeadingFootPosition.setToZero(footstepSoleFrame);
         tempTrailingFootPosition.setToZero(trailingFootFrame);
         tempLeadingFootPosition.changeFrame(trailingFootFrame);

         toLeadingFoot.setToZero(trailingFootFrame);
         toLeadingFoot.set(tempLeadingFootPosition);
         toLeadingFoot.sub(tempTrailingFootPosition);

         icpProximityToLeadingFootForDSToeOff.set(icpPercentOfStanceForDSToeOff.getDoubleValue() * toLeadingFoot.length());

         return icpProximityToLeadingFootForDSToeOff.getDoubleValue();
      }
   }

   private boolean evaluateLineToeOffConditions(RobotSide trailingLeg)
   {
      return evaluateToeOffConditions(trailingLeg, doLineToeOff, computeToeLineContact, updateLineContactDuringToeOff.getBooleanValue());
   }

   private boolean evaluatePointToeOffConditions(RobotSide trailingLeg)
   {
      return evaluateToeOffConditions(trailingLeg, doPointToeOff, computeToePointContact, updatePointContactDuringToeOff.getBooleanValue());
   }

   private boolean evaluateToeOffConditions(RobotSide trailingLeg, BooleanYoVariable doToeOff, BooleanYoVariable computeToePoints, boolean updateDuringToeOff)
   {
      if (!this.isDesiredICPOKForToeOffFilt.getBooleanValue() || !this.isCurrentICPOKForToeOffFilt.getBooleanValue())
      {
         doToeOff.set(false);
         computeToePoints.set(true);
         return false;
      }

      needToSwitchToToeOffForAnkleLimit.set(checkAnkleLimitForToeOff(trailingLeg));
      if (needToSwitchToToeOffForAnkleLimit.getBooleanValue())
      {
         doToeOff.set(true);
         computeToePoints.set(updateDuringToeOff);
         return false;
      }

      if (!isDesiredECMPOKForToeOffFilt.getBooleanValue())
      {
         doToeOff.set(false);
         computeToePoints.set(true);
         return false;
      }

      return true;
   }

   private boolean checkAnkleLimitForToeOff(RobotSide trailingLeg)
   {
      OneDoFJoint anklePitch = fullRobotModel.getLegJoint(trailingLeg, LegJointName.ANKLE_PITCH);
      double lowerLimit = Math.max(anklePitch.getJointLimitLower() + 0.02, ankleLowerLimitToTriggerToeOff.getDoubleValue());
      isRearAnklePitchHittingLimit.set(anklePitch.getQ() < lowerLimit);
      isRearAnklePitchHittingLimitFilt.update();

      if (!doToeOffWhenHittingAnkleLimit.getBooleanValue())
         return false;

      if (!isDesiredICPOKForToeOffFilt.getBooleanValue() || !isCurrentICPOKForToeOffFilt.getBooleanValue())
         return false;

      return isRearAnklePitchHittingLimitFilt.getBooleanValue();
   }

   private void isReadyToSwitchToLineToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      if (!isFrontFootWellPositionedForToeOff(trailingLeg, frontFootFrame))
      {
         doLineToeOff.set(false);
         computeToeLineContact.set(true);
         return;
      }

      computeToeLineContact.set(updateLineContactDuringToeOff.getBooleanValue());
      doLineToeOff.set(true);
   }

   private void isReadyToSwitchToPointToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      if (!isFrontFootWellPositionedForToeOff(trailingLeg, frontFootFrame))
      {
         doPointToeOff.set(false);
         computeToePointContact.set(true);
         return;
      }

      computeToePointContact.set(updatePointContactDuringToeOff.getBooleanValue());
      doPointToeOff.set(true);
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

   public boolean canDoToeOffSingleSupoprt(Footstep nextFootstep, RobotSide transferToSide)
   {
      if (!doToeOffIfPossibleInSingleSupport.getBooleanValue())
         return false;

      return canDoToeOff(nextFootstep, transferToSide);
   }

   public boolean canDoToeOffDoubleSupport(Footstep nextFootstep, RobotSide transferToSide)
   {
      if (!doToeOffIfPossibleInDoubleSupport.getBooleanValue())
         return false;

      return canDoToeOff(nextFootstep, transferToSide);
   }

   public boolean canDoToeOff(Footstep nextFootstep, RobotSide transferToSide)
   {
      RobotSide nextTrailingLeg = transferToSide.getOppositeSide();
      ReferenceFrame nextFrontFootFrame;
      if (nextFootstep != null)
         nextFrontFootFrame = nextFootstep.getPoseReferenceFrame();
      else
         nextFrontFootFrame = feet.get(nextTrailingLeg.getOppositeSide()).getFrameAfterParentJoint();

      return isFrontFootWellPositionedForToeOff(nextTrailingLeg, nextFrontFootFrame);
   }

   public boolean doLineToeOff()
   {
      return doLineToeOff.getBooleanValue();
   }

   public boolean doPointToeOff()
   {
      return doPointToeOff.getBooleanValue();
   }

   public boolean doToeOffIfPossibleInDoubleSupport()
   {
      return doToeOffIfPossibleInDoubleSupport.getBooleanValue();
   }

   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return doToeOffIfPossibleInSingleSupport.getBooleanValue();
   }

   public boolean shouldComputeToeLineContact()
   {
      return computeToeLineContact.getBooleanValue();
   }

   public boolean shouldComputeToePointContact()
   {
      return computeToePointContact.getBooleanValue();
   }

   public void setDoToeOffIfPossibleInDoubleSupport(boolean doToeOff)
   {
      doToeOffIfPossibleInDoubleSupport.set(doToeOff);
   }

   public double getExtraCoMMaxHeightWithToes()
   {
      return extraCoMMaxHeightWithToes.getDoubleValue();
   }

   public void reset()
   {
      isDesiredECMPOKForToeOff.set(false);
      isDesiredECMPOKForToeOffFilt.set(false);

      isRearAnklePitchHittingLimit.set(false);
      isRearAnklePitchHittingLimitFilt.set(false);

      isDesiredICPOKForToeOff.set(false);
      isDesiredICPOKForToeOffFilt.set(false);

      isCurrentICPOKForToeOff.set(false);
      isCurrentICPOKForToeOffFilt.set(false);

      computeToeLineContact.set(true);
      computeToePointContact.set(true);

      doLineToeOff.set(false);
      doPointToeOff.set(false);
   }

   private void updateOnLineToeSupportPolygon(FramePoint exitCMP, FramePoint2d desiredECMP, RobotSide trailingSide, FrameConvexPolygon2d leadingFootSupportPolygon)
   {
      if (exitCMP == null)
         computeToeContacts(trailingSide);
      else
         computeToeLineContacts(exitCMP, desiredECMP, trailingSide);

      onToesSupportPolygon.setIncludingFrameAndUpdate(leadingFootSupportPolygon);
      onToesSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      toeOffLine.getFirstEndpoint(tmpPoint2d);
      onToesSupportPolygon.addVertexChangeFrameAndProjectToXYPlane(tmpPoint2d);
      toeOffLine.getSecondEndpoint(tmpPoint2d);
      onToesSupportPolygon.addVertexChangeFrameAndProjectToXYPlane(tmpPoint2d);

      onToesSupportPolygon.update();
   }

   private void updateOnPointToeSupportPolygon(FramePoint exitCMP, FramePoint2d desiredECMP, RobotSide trailingSide, FrameConvexPolygon2d leadingFootSupportPolygon)
   {
      if (exitCMP == null)
         computeToeContacts(trailingSide);
      else
         computeToePointContacts(exitCMP, desiredECMP, trailingSide);

      onToesSupportPolygon.setIncludingFrameAndUpdate(leadingFootSupportPolygon);
      onToesSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      onToesSupportPolygon.addVertexChangeFrameAndProjectToXYPlane(toeOffPoint);
      onToesSupportPolygon.update();
   }

   private void computeToePointContacts(FramePoint exitCMP, FramePoint2d desiredECMP, RobotSide supportSide)
   {
      toeOffCalculator.setExitCMP(exitCMP, supportSide);
      toeOffCalculator.computeToeOffContactPoint(desiredECMP, supportSide);

      toeOffPoint.setToZero(feet.get(supportSide).getSoleFrame());
      toeOffCalculator.getToeOffContactPoint(toeOffPoint, supportSide);
   }

   private void computeToeLineContacts(FramePoint exitCMP, FramePoint2d desiredECMP, RobotSide supportSide)
   {
      toeOffCalculator.setExitCMP(exitCMP, supportSide);
      toeOffCalculator.computeToeOffContactLine(desiredECMP, supportSide);

      toeOffLine.setToZero(feet.get(supportSide).getSoleFrame());
      toeOffCalculator.getToeOffContactLine(toeOffLine, supportSide);
   }

   private void computeToeContacts(RobotSide supportSide)
   {
      FrameConvexPolygon2d footDefaultPolygon = footDefaultPolygons.get(supportSide);
      ReferenceFrame referenceFrame = footDefaultPolygon.getReferenceFrame();
      toeOffLine.setFirstEndpoint(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);
      toeOffLine.setSecondEndpoint(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);

      // gets the leading two toe points
      for (int i = 0; i < footDefaultPolygon.getNumberOfVertices(); i++)
      {
         footDefaultPolygon.getFrameVertex(i, tmpPoint2d);
         if (tmpPoint2d.getX() > toeOffLine.getFirstEndpoint().getX())
         { // further ahead than leading point
            toeOffLine.setSecondEndpoint(referenceFrame, toeOffLine.getFirstEndpoint());
            toeOffLine.setFirstEndpoint(tmpPoint2d);
         }
         else if (tmpPoint2d.getX() > toeOffLine.getSecondEndpoint().getX())
         { // further ahead than second leading point
            toeOffLine.setSecondEndpoint(tmpPoint2d);
         }
      }

      toeOffPoint.setToZero(footDefaultPolygon.getReferenceFrame());
      toeOffPoint.set(toeOffLine.midpoint());
   }
}