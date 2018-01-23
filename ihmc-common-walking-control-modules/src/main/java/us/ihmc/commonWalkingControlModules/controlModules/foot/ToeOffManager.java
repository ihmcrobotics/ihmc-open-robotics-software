package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ToeOffManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean DO_TOE_OFF_FOR_SIDE_STEPS = false;
   private static final boolean ENABLE_TOE_OFF_FOR_STEP_DOWN = true;

   private static final double forwardSteppingThreshold = -0.05;
   private static final double stepDownTooFarForToeOff = -0.10;
   private static final double minimumAngleForSideStepping = 45.0;
   private static final double extraCoMHeightWithToes = 0.08;

   private static final int largeGlitchWindowSize = 10;
   private static final int smallGlitchWindowSize = 2;

   private final YoBoolean doToeOffIfPossibleInDoubleSupport = new YoBoolean("doToeOffIfPossibleInDoubleSupport", registry);
   private final YoBoolean doToeOffIfPossibleInSingleSupport = new YoBoolean("doToeOffIfPossibleInSingleSupport", registry);
   private final YoBoolean doToeOffWhenHittingAnkleLimit = new YoBoolean("doToeOffWhenHittingAnkleLimit", registry);
   private final YoBoolean doToeOffWhenHittingKneeLimit = new YoBoolean("doToeOffWhenHittingKneeLimit", registry);
   private final YoBoolean doPointToeOff = new YoBoolean("doPointToeOff", registry);
   private final YoBoolean doLineToeOff = new YoBoolean("doLineToeOff", registry);

   private final YoBoolean useToeLineContactInSwing = new YoBoolean("useToeLineContactInSwing", registry);
   private final YoBoolean useToeLineContactInTransfer = new YoBoolean("useToeLineContactInTransfer", registry);
   private final YoBoolean computeToeLineContact = new YoBoolean("computeToeLineContact", registry);
   private final YoBoolean computeToePointContact = new YoBoolean("computeToePointContact", registry);
   private final YoBoolean updateLineContactDuringToeOff = new YoBoolean("updateLineContactDuringToeOff", registry);
   private final YoBoolean updatePointContactDuringToeOff = new YoBoolean("updatePointContactDuringToeOff", registry);

   private final YoDouble ankleLowerLimitToTriggerToeOff = new YoDouble("ankleLowerLimitToTriggerToeOff", registry);
   private final YoDouble kneeUpperLimitToTriggerToeOff = new YoDouble("kneeUpperLimitToTriggerToeOff", registry);
   private final YoDouble icpPercentOfStanceForDSToeOff = new YoDouble("icpPercentOfStanceForDSToeOff", registry);
   private final YoDouble icpPercentOfStanceForSSToeOff = new YoDouble("icpPercentOfStanceForSSToeOff", registry);
   private final YoDouble icpProximityForDSToeOff = new YoDouble("icpProximityForDSToeOff", registry);
   private final YoDouble icpProximityForSSToeOff = new YoDouble("icpProximityForSSToeOff", registry);
   private final YoDouble ecmpProximityForToeOff = new YoDouble("ecmpProximityForToeOff", registry);
   private final YoDouble copProximityForToeOff = new YoDouble("copProximityForToeOff", registry);

   private final YoBoolean isDesiredICPOKForToeOff = new YoBoolean("isDesiredICPOKForToeOff", registry);
   private final YoBoolean isCurrentICPOKForToeOff = new YoBoolean("isCurrentICPOKForToeOff", registry);
   private final YoBoolean isDesiredECMPOKForToeOff = new YoBoolean("isDesiredECMPOKForToeOff", registry);
   private final YoBoolean isDesiredCoPOKForToeOff = new YoBoolean("isDesiredCoPOKForToeOff", registry);
   private final YoBoolean isFrontFootWellPositionedForToeOff = new YoBoolean("isFrontFootWellPositionedForToeOff", registry);

   private final YoBoolean needToSwitchToToeOffForJointLimit = new YoBoolean("needToSwitchToToeOffForJointLimit", registry);
   private final YoBoolean isRearAnklePitchHittingLimit = new YoBoolean("isRearAnklePitchHittingLimit", registry);
   private final YoBoolean isRearKneePitchHittingLimit = new YoBoolean("isRearKneePitchHittingLimit", registry);

   private final GlitchFilteredYoBoolean isDesiredICPOKForToeOffFilt = new GlitchFilteredYoBoolean("isDesiredICPOKForToeOffFilt",
         registry, isDesiredICPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isCurrentICPOKForToeOffFilt = new GlitchFilteredYoBoolean("isCurrentICPOKForToeOffFilt",
         registry, isCurrentICPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isDesiredECMPOKForToeOffFilt = new GlitchFilteredYoBoolean("isDesiredECMPOKForToeOffFilt",
         registry, isDesiredECMPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isDesiredCoPOKForToeOffFilt = new GlitchFilteredYoBoolean("isDesiredCoPOKForToeOffFilt",
         registry, isDesiredCoPOKForToeOff, smallGlitchWindowSize);

   private final GlitchFilteredYoBoolean isRearAnklePitchHittingLimitFilt = new GlitchFilteredYoBoolean("isRearAnklePitchHittingLimitFilt",
         registry, isRearAnklePitchHittingLimit, largeGlitchWindowSize);
   private final GlitchFilteredYoBoolean isRearKneePitchHittingLimitFilt = new GlitchFilteredYoBoolean("isRearKneePitchHittingLimitFilt",
                                                                                                        registry, isRearKneePitchHittingLimit, largeGlitchWindowSize);

   private final YoDouble minStepLengthForToeOff = new YoDouble("minStepLengthForToeOff", registry);
   private final YoDouble minStepHeightForToeOff = new YoDouble("minStepHeightForToeOff", registry);
   private final YoDouble extraCoMMaxHeightWithToes = new YoDouble("extraCoMMaxHeightWithToes", registry);


   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final List<FramePoint3D> contactStatePoints = new ArrayList<>();

   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final SideDependentList<FrameConvexPolygon2d> footDefaultPolygons;
   private final FrameConvexPolygon2d leadingFootSupportPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d onToesSupportPolygon = new FrameConvexPolygon2d();

   private final FramePoint2D tempLeadingFootPosition = new FramePoint2D();
   private final FramePoint2D tempTrailingFootPosition = new FramePoint2D();
   private final FramePoint3D tempLeadingFootPositionInWorld = new FramePoint3D();
   private final FramePoint3D tempTrailingFootPositionInWorld = new FramePoint3D();
   private final FrameVector2D toLeadingFoot = new FrameVector2D();

   private final HashMap<ToeContact, AbstractToeContact> toeContacts = new HashMap<>();

   private Footstep nextFootstep;

   private final ToeOffParameters toeOffParameters;

   private final FullHumanoidRobotModel fullRobotModel;
   private final ToeOffCalculator toeOffCalculator;

   private final double inPlaceWidth;
   private final double footLength;

   public ToeOffManager(HighLevelHumanoidControllerToolbox controllerToolbox, ToeOffCalculator toeOffCalculator,
         WalkingControllerParameters walkingControllerParameters, SideDependentList<? extends ContactablePlaneBody> feet, YoVariableRegistry parentRegistry)
   {
      this(controllerToolbox.getFullRobotModel(), toeOffCalculator, walkingControllerParameters, feet, createFootContactStates(controllerToolbox), parentRegistry);
   }

   public ToeOffManager(FullHumanoidRobotModel fullRobotModel, ToeOffCalculator toeOffCalculator,
         WalkingControllerParameters walkingControllerParameters, SideDependentList<? extends ContactablePlaneBody> feet,
         SideDependentList<YoPlaneContactState> footContactStates, YoVariableRegistry parentRegistry)
   {
      this.toeOffParameters = walkingControllerParameters.getToeOffParameters();
      this.doToeOffIfPossibleInDoubleSupport.set(toeOffParameters.doToeOffIfPossible());
      this.doToeOffIfPossibleInSingleSupport.set(toeOffParameters.doToeOffIfPossibleInSingleSupport());

      this.doToeOffWhenHittingAnkleLimit.set(toeOffParameters.doToeOffWhenHittingAnkleLimit());
      this.doToeOffWhenHittingKneeLimit.set(toeOffParameters.doToeOffWhenHittingKneeLimit());

      this.ankleLowerLimitToTriggerToeOff.set(toeOffParameters.getAnkleLowerLimitToTriggerToeOff());
      this.kneeUpperLimitToTriggerToeOff.set(toeOffParameters.getKneeUpperLimitToTriggerToeOff());

      this.icpPercentOfStanceForDSToeOff.set(toeOffParameters.getICPPercentOfStanceForDSToeOff());
      this.icpPercentOfStanceForSSToeOff.set(toeOffParameters.getICPPercentOfStanceForSSToeOff());

      this.ecmpProximityForToeOff.set(toeOffParameters.getECMPProximityForToeOff());
      this.copProximityForToeOff.set(toeOffParameters.getCoPProximityForToeOff());

      this.toeOffCalculator = toeOffCalculator;

      this.fullRobotModel = fullRobotModel;
      this.feet = feet;

      this.inPlaceWidth = walkingControllerParameters.getSteppingParameters().getInPlaceWidth();
      this.footLength = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset()
            + walkingControllerParameters.getSteppingParameters().getFootForwardOffset();

      extraCoMMaxHeightWithToes.set(extraCoMHeightWithToes);

      minStepLengthForToeOff.set(toeOffParameters.getMinStepLengthForToeOff());
      minStepHeightForToeOff.set(toeOffParameters.getMinStepHeightForToeOff());

      useToeLineContactInSwing.set(toeOffParameters.useToeOffLineContactInSwing());
      useToeLineContactInTransfer.set(toeOffParameters.useToeOffLineContactInTransfer());

      updateLineContactDuringToeOff.set(toeOffParameters.updateLineContactDuringToeOff());
      updatePointContactDuringToeOff.set(toeOffParameters.updatePointContactDuringToeOff());

      footDefaultPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         footDefaultPolygons.put(robotSide, new FrameConvexPolygon2d(feet.get(robotSide).getContactPoints2d()));
      }

      this.footContactStates = footContactStates;

      toeContacts.put(ToeContact.LINE, new ToeLineContact());
      toeContacts.put(ToeContact.POINT, new ToePointContact());

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

   public void reset()
   {
      isDesiredECMPOKForToeOff.set(false);
      isDesiredECMPOKForToeOffFilt.set(false);

      isDesiredCoPOKForToeOff.set(false);
      isDesiredCoPOKForToeOffFilt.set(false);

      isRearAnklePitchHittingLimit.set(false);
      isRearAnklePitchHittingLimitFilt.set(false);

      isDesiredICPOKForToeOff.set(false);
      isDesiredICPOKForToeOffFilt.set(false);

      isCurrentICPOKForToeOff.set(false);
      isCurrentICPOKForToeOffFilt.set(false);

      isFrontFootWellPositionedForToeOff.set(false);
      computeToeLineContact.set(true);
      computeToePointContact.set(true);

      doLineToeOff.set(false);
      doPointToeOff.set(false);
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
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in double support, and sets the {@link ToeOffManager#doLineToeOff} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    *   <li>doToeOffIfPossibleInDoubleSupport</li>
    *   <li>desiredECMP location being within the support polygon account for toe-off, if {@link ToeOffParameters#checkECMPLocationToTriggerToeOff()} is true.</li>
    *   <li>desiredICP location being within the leading foot base of support.</li>
    *   <li>currentICP location being within the leading foot base of support.</li>
    *   <li>needToSwitchToToeOffForAnkleLimit</li>
    * </ol>
    * <p>
    * If able and the ankles are at the joint limits, transitions to toe-off. Then checks the current state being with the base of support. Then checks the
    * positioning of the leading leg to determine if it is acceptable.
    * </p>
    *
    * @param desiredECMP current desired ECMP from ICP feedback.
    * @param desiredICP current desired ICP from the reference trajectory.
    * @param currentICP current ICP based on the robot state.
    */
   public void updateToeOffStatusSingleSupport(FramePoint3D exitCMP, FramePoint2D desiredECMP, FramePoint2D desiredCoP, FramePoint2D desiredICP, FramePoint2D currentICP)
   {
      if (!doToeOffIfPossibleInSingleSupport.getBooleanValue())
      {
         doLineToeOff.set(false);
         doPointToeOff.set(false);

         isDesiredICPOKForToeOff.set(false);
         isDesiredICPOKForToeOffFilt.set(false);

         isCurrentICPOKForToeOff.set(false);
         isCurrentICPOKForToeOffFilt.set(false);

         isDesiredECMPOKForToeOff.set(false);
         isDesiredECMPOKForToeOffFilt.set(false);

         isDesiredCoPOKForToeOff.set(false);
         isDesiredCoPOKForToeOffFilt.set(false);

         isFrontFootWellPositionedForToeOff.set(false);
         computeToePointContact.set(false);
         computeToeLineContact.set(false);
         return;
      }

      RobotSide trailingLeg = nextFootstep.getRobotSide().getOppositeSide();
      double percentProximity = icpPercentOfStanceForSSToeOff.getDoubleValue();
      setLeadingFootSupportPolygonFromNextFootstep();

      AbstractToeContact toeContact;
      if (useToeLineContactInSwing.getBooleanValue())
      {
         computeToePointContact.set(false);
         toeContact = toeContacts.get(ToeContact.LINE);
      }
      else
      {
         computeToeLineContact.set(false);
         toeContact = toeContacts.get(ToeContact.POINT);
      }

      toeContact.updateToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, leadingFootSupportPolygon);

      ReferenceFrame soleFrame = nextFootstep.getSoleReferenceFrame();
      double requiredProximity = checkICPLocations(trailingLeg, desiredICP, currentICP, toeContact.getToeOffPoint(), leadingFootSupportPolygon, soleFrame, percentProximity);
      icpProximityForSSToeOff.set(requiredProximity);

      checkCoPLocation(desiredCoP);
      checkECMPLocation(desiredECMP);

      if (!toeContact.evaluateToeOffConditions(trailingLeg))
         return;

      toeContact.isReadyToSwitchToToeOff(trailingLeg, soleFrame);
   }

   /**
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in double support, and sets the {@link ToeOffManager#doLineToeOff} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    *   <li>doToeOffIfPossibleInDoubleSupport</li>
    *   <li>desiredECMP location being within the support polygon account for toe-off, if {@link ToeOffParameters#checkECMPLocationToTriggerToeOff()} is true.</li>
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
   public void updateToeOffStatusDoubleSupport(RobotSide trailingLeg, FramePoint3D exitCMP, FramePoint2D desiredECMP, FramePoint2D desiredCoP,
         FramePoint2D desiredICP, FramePoint2D currentICP)
   {
      nextFootstep = null;
      if (!doToeOffIfPossibleInDoubleSupport.getBooleanValue())
      {
         doLineToeOff.set(false);
         doPointToeOff.set(false);

         isDesiredICPOKForToeOff.set(false);
         isDesiredICPOKForToeOffFilt.set(false);

         isCurrentICPOKForToeOff.set(false);
         isCurrentICPOKForToeOffFilt.set(false);

         isDesiredECMPOKForToeOff.set(false);
         isDesiredECMPOKForToeOffFilt.set(false);

         isDesiredCoPOKForToeOff.set(false);
         isDesiredCoPOKForToeOffFilt.set(false);

         needToSwitchToToeOffForJointLimit.set(false);
         computeToePointContact.set(false);
         computeToeLineContact.set(false);
         return;
      }

      setLeadingFootSupportPolygon(trailingLeg);

      double percentProximity = icpPercentOfStanceForDSToeOff.getDoubleValue();

      AbstractToeContact toeContact;
      if (useToeLineContactInTransfer.getBooleanValue())
      {
         computeToePointContact.set(false);
         toeContact = toeContacts.get(ToeContact.LINE);
      }
      else
      {
         computeToeLineContact.set(false);
         toeContact = toeContacts.get(ToeContact.POINT);
      }

      toeContact.updateToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, leadingFootSupportPolygon);

      ReferenceFrame soleFrame = feet.get(trailingLeg.getOppositeSide()).getSoleFrame();
      double requiredProximity = checkICPLocations(trailingLeg, desiredICP, currentICP, toeContact.getToeOffPoint(), leadingFootSupportPolygon, soleFrame, percentProximity);
      icpProximityForDSToeOff.set(requiredProximity);

      checkCoPLocation(desiredCoP);
      checkECMPLocation(desiredECMP);

      if (!toeContact.evaluateToeOffConditions(trailingLeg))
         return;

      toeContact.isReadyToSwitchToToeOff(trailingLeg, soleFrame);
   }

   /**
    * Sets the support polygon of the location of the leading support foot. Only works during transfer
    * @param trailingLeg trailing leg
    */
   private void setLeadingFootSupportPolygon(RobotSide trailingLeg)
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
   }

   /**
    * Sets the predicted support polygon of the leading support foot from the next footstep.
    */
   private void setLeadingFootSupportPolygonFromNextFootstep()
   {
      if (nextFootstep == null)
         throw new RuntimeException("The next footstep has not been set.");

      ReferenceFrame footstepSoleFrame = nextFootstep.getSoleReferenceFrame();
      List<Point2D> predictedContactPoints = nextFootstep.getPredictedContactPoints();
      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         leadingFootSupportPolygon.setIncludingFrameAndUpdate(footstepSoleFrame, predictedContactPoints);
      }
      else
      {
         ConvexPolygon2D footPolygon = footDefaultPolygons.get(nextFootstep.getRobotSide()).getConvexPolygon2d();
         leadingFootSupportPolygon.setIncludingFrameAndUpdate(footstepSoleFrame, footPolygon);
      }
      leadingFootSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);
   }

   private void checkECMPLocation(FramePoint2D desiredECMP)
   {
      if (toeOffParameters.checkECMPLocationToTriggerToeOff())
      {
         desiredECMP.changeFrameAndProjectToXYPlane(onToesSupportPolygon.getReferenceFrame());
         isDesiredECMPOKForToeOff.set(onToesSupportPolygon.distance(desiredECMP) <= ecmpProximityForToeOff.getDoubleValue());
         isDesiredECMPOKForToeOffFilt.update();
      }
      else
      {
         isDesiredECMPOKForToeOff.set(true);
         isDesiredECMPOKForToeOffFilt.set(true);
      }
   }

   private void checkCoPLocation(FramePoint2D desiredCoP)
   {
      if (toeOffParameters.checkCoPLocationToTriggerToeOff())
      {
         desiredCoP.changeFrameAndProjectToXYPlane(onToesSupportPolygon.getReferenceFrame());
         isDesiredCoPOKForToeOff.set(onToesSupportPolygon.distance(desiredCoP) <= copProximityForToeOff.getDoubleValue());
         isDesiredCoPOKForToeOffFilt.update();
      }
      else
      {
         isDesiredCoPOKForToeOff.set(true);
         isDesiredCoPOKForToeOffFilt.set(true);
      }
   }

   private double checkICPLocations(RobotSide trailingLeg, FramePoint2D desiredICP, FramePoint2D currentICP, FramePoint2D toeOffPoint,
         FrameConvexPolygon2d leadingFootSupportPolygon, ReferenceFrame nextSoleFrame, double percentProximity)
   {
      boolean isDesiredICPOKForToeOff, isCurrentICPOKForToeOff;
      double requiredProximity;
      if (percentProximity > 0.0)
      {
         // compute stance length
         requiredProximity = computeRequiredICPProximity(trailingLeg, nextSoleFrame, toeOffPoint, percentProximity);

         isDesiredICPOKForToeOff =
               onToesSupportPolygon.isPointInside(desiredICP) && leadingFootSupportPolygon.distance(desiredICP) < requiredProximity;
         isCurrentICPOKForToeOff =
               onToesSupportPolygon.isPointInside(currentICP) && leadingFootSupportPolygon.distance(currentICP) < requiredProximity;
      }
      else
      {
         requiredProximity = 0.0;
         isDesiredICPOKForToeOff = leadingFootSupportPolygon.isPointInside(desiredICP);
         isCurrentICPOKForToeOff = leadingFootSupportPolygon.isPointInside(currentICP);
      }

      this.isCurrentICPOKForToeOff.set(isCurrentICPOKForToeOff);
      this.isDesiredICPOKForToeOff.set(isDesiredICPOKForToeOff);
      this.isCurrentICPOKForToeOffFilt.update();
      this.isDesiredICPOKForToeOffFilt.update();

      return requiredProximity;
   }

   private double computeRequiredICPProximity(RobotSide trailingLeg, ReferenceFrame nextSoleFrame, FramePoint2D toeOffPoint, double percentOfStanceForToeOff)
   {
      ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getSoleFrame();
      tempLeadingFootPosition.setToZero(nextSoleFrame);
      tempLeadingFootPosition.changeFrameAndProjectToXYPlane(trailingFootFrame);
//      tempTrailingFootPosition.setToZero(trailingFootFrame);
      toeOffPoint.changeFrameAndProjectToXYPlane(trailingFootFrame);

      toLeadingFoot.setToZero(trailingFootFrame);
      toLeadingFoot.set(tempLeadingFootPosition);
      toLeadingFoot.sub(toeOffPoint);

      return percentOfStanceForToeOff * toLeadingFoot.length();
   }

   private boolean checkAnkleLimitForToeOff(RobotSide trailingLeg)
   {
      OneDoFJoint anklePitch = fullRobotModel.getLegJoint(trailingLeg, LegJointName.ANKLE_PITCH);
      double lowerLimit = Math.max(anklePitch.getJointLimitLower() + 0.02, ankleLowerLimitToTriggerToeOff.getDoubleValue()); // todo extract variable
      isRearAnklePitchHittingLimit.set(anklePitch.getQ() < lowerLimit);
      isRearAnklePitchHittingLimitFilt.update();

      if (!doToeOffWhenHittingAnkleLimit.getBooleanValue() || !isDesiredICPOKForToeOffFilt.getBooleanValue() || !isCurrentICPOKForToeOffFilt.getBooleanValue())
         return false;

      return isRearAnklePitchHittingLimitFilt.getBooleanValue();
   }

   private boolean checkKneeLimitForToeOff(RobotSide leadingLeg)
   {
      OneDoFJoint kneePitch = fullRobotModel.getLegJoint(leadingLeg, LegJointName.KNEE_PITCH);
      double upperLimit = Math.min(kneePitch.getJointLimitUpper() - 0.02, kneeUpperLimitToTriggerToeOff.getDoubleValue()); // todo extract variable
      isRearKneePitchHittingLimit.set(kneePitch.getQ() > upperLimit);
      isRearKneePitchHittingLimitFilt.update();

      if (!doToeOffWhenHittingKneeLimit.getBooleanValue() || !isDesiredICPOKForToeOffFilt.getBooleanValue() || !isCurrentICPOKForToeOffFilt.getBooleanValue())
         return false;

      return isRearKneePitchHittingLimitFilt.getBooleanValue();
   }

   private boolean isFrontFootWellPositionedForToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getSoleFrame();
      tempTrailingFootPosition.setToZero(trailingFootFrame);
      tempLeadingFootPosition.setToZero(frontFootFrame);
      tempLeadingFootPosition.changeFrameAndProjectToXYPlane(trailingFootFrame);

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

      boolean isForwardStepping = tempLeadingFootPosition.getX() > forwardSteppingThreshold;
      if (!isForwardStepping)
         return false;

      if (ENABLE_TOE_OFF_FOR_STEP_DOWN)
      {
         boolean isNextStepLowEnough = stepHeight < -minStepHeightForToeOff.getDoubleValue();
         if (isNextStepLowEnough)
            return true;
      }
      else
      {
         boolean isNextStepTooLow = stepHeight < stepDownTooFarForToeOff;
         if (isNextStepTooLow)
            return false;
      }

      boolean isSideStepping = Math.abs(Math.atan2(tempLeadingFootPosition.getY(), tempLeadingFootPosition.getX())) > Math.toRadians(minimumAngleForSideStepping);
      if (isSideStepping && !DO_TOE_OFF_FOR_SIDE_STEPS)
         return false;

      boolean isStepLongEnough = tempLeadingFootPosition.distance(tempTrailingFootPosition) > minStepLengthForToeOff.getDoubleValue();
      boolean isStepLongEnoughAlongX = tempLeadingFootPosition.getX() > footLength;
      return isStepLongEnough && isStepLongEnoughAlongX;
   }

   /**
    * Checks whether or not the next footstep in {@param nextFootstep} is in correct location to achieve toe off.
    * @param nextFootstep footstep to consider.
    * @param transferToSide upcoming support side.
    * @return whether or not the footstep location is ok.
    */
   public boolean canDoSingleSupportToeOff(Footstep nextFootstep, RobotSide transferToSide)
   {
      if (!doToeOffIfPossibleInSingleSupport.getBooleanValue())
         return false;

      return canDoToeOff(nextFootstep, transferToSide);
   }

   /**
    * Checks whether or not the next footstep in {@param nextFootstep} is in correct location to achieve toe off.
    * @param nextFootstep footstep to consider.
    * @param transferToSide upcoming support side.
    * @return whether or not the footstep location is ok.
    */
   public boolean canDoDoubleSupportToeOff(Footstep nextFootstep, RobotSide transferToSide)
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
         nextFrontFootFrame = nextFootstep.getSoleReferenceFrame();
      else
         nextFrontFootFrame = feet.get(nextTrailingLeg.getOppositeSide()).getSoleFrame();

      this.isFrontFootWellPositionedForToeOff.set(isFrontFootWellPositionedForToeOff(nextTrailingLeg, nextFrontFootFrame));
      return this.isFrontFootWellPositionedForToeOff.getBooleanValue();
   }

   public boolean doLineToeOff()
   {
      return doLineToeOff.getBooleanValue();
   }

   public boolean doPointToeOff()
   {
      return doPointToeOff.getBooleanValue();
   }

   public boolean shouldComputeToeLineContact()
   {
      return computeToeLineContact.getBooleanValue();
   }

   public boolean shouldComputeToePointContact()
   {
      return computeToePointContact.getBooleanValue();
   }

   public double getExtraCoMMaxHeightWithToes()
   {
      return extraCoMMaxHeightWithToes.getDoubleValue();
   }


   private enum ToeContact
   {
      POINT, LINE
   }

   private abstract class AbstractToeContact
   {
      protected final FrameLineSegment2D toeOffLine = new FrameLineSegment2D();
      protected final FramePoint2D toeOffPoint = new FramePoint2D();
      protected final FramePoint2D tmpPoint2d = new FramePoint2D();

      public abstract void updateToeSupportPolygon(FramePoint3D exitCMP, FramePoint2D desiredECMP, RobotSide trailingSide, FrameConvexPolygon2d leadingFootSupportPolygon);

      public abstract void isReadyToSwitchToToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame);

      public abstract boolean evaluateToeOffConditions(RobotSide trailingLeg);

      protected void computeToeContacts(RobotSide supportSide)
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
               toeOffLine.setSecondEndpoint(toeOffLine.getFirstEndpoint());
               toeOffLine.setFirstEndpoint(tmpPoint2d);
            }
            else if (tmpPoint2d.getX() > toeOffLine.getSecondEndpoint().getX())
            { // further ahead than second leading point
               toeOffLine.setSecondEndpoint(tmpPoint2d);
            }
         }

         toeOffPoint.setToZero(footDefaultPolygon.getReferenceFrame());
         toeOffLine.midpoint(toeOffPoint);
      }

      public FramePoint2D getToeOffPoint()
      {
         return toeOffPoint;
      }
   }

   private class ToeLineContact extends AbstractToeContact
   {
      @Override
      public void updateToeSupportPolygon(FramePoint3D exitCMP, FramePoint2D desiredECMP, RobotSide trailingSide, FrameConvexPolygon2d leadingFootSupportPolygon)
      {
         if (exitCMP == null)
            computeToeContacts(trailingSide);
         else
            computeToeContacts(exitCMP, desiredECMP, trailingSide);

         onToesSupportPolygon.setIncludingFrameAndUpdate(leadingFootSupportPolygon);
         onToesSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

         tmpPoint2d.setIncludingFrame(toeOffLine.getFirstEndpoint());
         onToesSupportPolygon.addVertexChangeFrameAndProjectToXYPlane(tmpPoint2d);
         tmpPoint2d.setIncludingFrame(toeOffLine.getSecondEndpoint());
         onToesSupportPolygon.addVertexChangeFrameAndProjectToXYPlane(tmpPoint2d);

         onToesSupportPolygon.update();

         toeOffLine.midpoint(toeOffPoint);
      }

      @Override
      public void isReadyToSwitchToToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
      {
         isFrontFootWellPositionedForToeOff.set(isFrontFootWellPositionedForToeOff(trailingLeg, frontFootFrame));
         if (!isFrontFootWellPositionedForToeOff.getBooleanValue())
         {
            doLineToeOff.set(false);
            computeToeLineContact.set(true);
            return;
         }

         computeToeLineContact.set(updateLineContactDuringToeOff.getBooleanValue());
         doLineToeOff.set(true);
      }

      private void computeToeContacts(FramePoint3D exitCMP, FramePoint2D desiredECMP, RobotSide supportSide)
      {
         toeOffCalculator.setExitCMP(exitCMP, supportSide);
         toeOffCalculator.computeToeOffContactLine(desiredECMP, supportSide);

         toeOffLine.setToZero(feet.get(supportSide).getSoleFrame());
         toeOffCalculator.getToeOffContactLine(toeOffLine, supportSide);
      }

      @Override
      public boolean evaluateToeOffConditions(RobotSide trailingLeg)
      {
         if (!isDesiredICPOKForToeOffFilt.getBooleanValue() || !isCurrentICPOKForToeOffFilt.getBooleanValue())
         {
            doLineToeOff.set(false);
            computeToeLineContact.set(true);
            return false;
         }

         boolean ankleAtLimit = checkAnkleLimitForToeOff(trailingLeg);
         boolean kneeAtLimit = checkKneeLimitForToeOff(trailingLeg.getOppositeSide());

         needToSwitchToToeOffForJointLimit.set(ankleAtLimit || kneeAtLimit);
         if (needToSwitchToToeOffForJointLimit.getBooleanValue())
         {
            doLineToeOff.set(true);
            computeToeLineContact.set(updateLineContactDuringToeOff.getBooleanValue());
            return false;
         }

         if (!isDesiredECMPOKForToeOffFilt.getBooleanValue() || !isDesiredCoPOKForToeOffFilt.getBooleanValue())
         {
            doLineToeOff.set(false);
            computeToeLineContact.set(true);
            return false;
         }

         return true;
      }
   }

   private class ToePointContact extends AbstractToeContact
   {
      @Override
      public void updateToeSupportPolygon(FramePoint3D exitCMP, FramePoint2D desiredECMP, RobotSide trailingSide, FrameConvexPolygon2d leadingFootSupportPolygon)
      {
         if (exitCMP == null)
            computeToeContacts(trailingSide);
         else
            computeToeContacts(exitCMP, desiredECMP, trailingSide);

         onToesSupportPolygon.setIncludingFrameAndUpdate(leadingFootSupportPolygon);
         onToesSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

         onToesSupportPolygon.addVertexChangeFrameAndProjectToXYPlane(toeOffPoint);
         onToesSupportPolygon.update();
      }

      @Override
      public void isReadyToSwitchToToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
      {
         isFrontFootWellPositionedForToeOff.set(isFrontFootWellPositionedForToeOff(trailingLeg, frontFootFrame));
         if (!isFrontFootWellPositionedForToeOff.getBooleanValue())
         {
            doPointToeOff.set(false);
            computeToePointContact.set(true);
            return;
         }

         computeToePointContact.set(updatePointContactDuringToeOff.getBooleanValue());
         doPointToeOff.set(true);
      }

      private void computeToeContacts(FramePoint3D exitCMP, FramePoint2D desiredECMP, RobotSide supportSide)
      {
         toeOffCalculator.setExitCMP(exitCMP, supportSide);
         toeOffCalculator.computeToeOffContactPoint(desiredECMP, supportSide);

         toeOffPoint.setToZero(feet.get(supportSide).getSoleFrame());
         toeOffCalculator.getToeOffContactPoint(toeOffPoint, supportSide);
      }

      @Override
      public boolean evaluateToeOffConditions(RobotSide trailingLeg)
      {
         if (!isDesiredICPOKForToeOffFilt.getBooleanValue() || !isCurrentICPOKForToeOffFilt.getBooleanValue())
         {
            doPointToeOff.set(false);
            computeToePointContact.set(true);
            return false;
         }

         boolean ankleAtLimit = checkAnkleLimitForToeOff(trailingLeg);
         boolean kneeAtLimit = checkKneeLimitForToeOff(trailingLeg.getOppositeSide());
         needToSwitchToToeOffForJointLimit.set(ankleAtLimit || kneeAtLimit);
         if (needToSwitchToToeOffForJointLimit.getBooleanValue())
         {
            doPointToeOff.set(true);
            computeToePointContact.set(updatePointContactDuringToeOff.getBooleanValue());
            return false;
         }

         // I don't care about the CoP location during transfer
         if (!isDesiredECMPOKForToeOffFilt.getBooleanValue())
         {
            doPointToeOff.set(false);
            computeToePointContact.set(true);
            return false;
         }

         return true;
      }
   }
}