package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class ToeOffManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean DO_TOE_OFF_FOR_SIDE_STEPS = true;
   private static final boolean ENABLE_TOE_OFF_FOR_STEP_DOWN = true;

   private static final double forwardSteppingThreshold = -0.05;
   private static final double stepDownTooFarForToeOff = -0.10;
   private static final double minimumAngleForSideStepping = 45.0;
   private static final double extraCoMHeightWithToes = 0.08;

   private static final int largeGlitchWindowSize = 10;
   private static final int smallGlitchWindowSize = 2;

   private final BooleanProvider doToeOffIfPossibleInDoubleSupport;
   private final BooleanProvider doToeOffIfPossibleInSingleSupport;
   private final BooleanProvider doToeOffWhenHittingAnkleLimit;
   private final BooleanProvider doToeOffWhenHittingLeadingKneeUpperLimit;
   private final BooleanProvider doToeOffWhenHittingRearKneeLowerLimit;

   private final YoBoolean doPointToeOff = new YoBoolean("doPointToeOff", registry);
   private final YoBoolean doLineToeOff = new YoBoolean("doLineToeOff", registry);

   private final BooleanProvider useToeLineContactInSwing;
   private final BooleanProvider useToeLineContactInTransfer;
   private final YoBoolean computeToeLineContact = new YoBoolean("computeToeLineContact", registry);
   private final YoBoolean computeToePointContact = new YoBoolean("computeToePointContact", registry);

   private final BooleanProvider updateLineContactDuringToeOff;
   private final BooleanProvider updatePointContactDuringToeOff;
   private final BooleanProvider checkECMPForToeOff;
   private final BooleanProvider checkCoPForToeOff;

   private final BooleanProvider lookAtTwoStepCapturabilityForToeOff;

   private final DoubleProvider ankleLowerLimitToTriggerToeOff;
   private final DoubleProvider kneeUpperLimitToTriggerToeOff;
   private final DoubleProvider kneeLowerLimitToTriggerToeOff;
   private final DoubleProvider icpPercentOfStanceForDSToeOff;
   private final DoubleProvider icpPercentOfStanceForSSToeOff;

   private final YoDouble icpProximityToLeadingFootForDSToeOff = new YoDouble("icpProximityToLeadingFootForDSToeOff", registry);
   private final YoDouble icpProximityToLeadingFootForSSToeOff = new YoDouble("icpProximityToLeadingFootForSSToeOff", registry);
   private final DoubleProvider icpProximityForToeOff;
   private final DoubleProvider ecmpProximityForToeOff;
   private final DoubleProvider copProximityForToeOff;

   private final YoBoolean isDesiredICPOKForToeOff = new YoBoolean("isDesiredICPOKForToeOff", registry);
   private final YoBoolean isCurrentICPOKForToeOff = new YoBoolean("isCurrentICPOKForToeOff", registry);
   private final YoBoolean isDesiredECMPOKForToeOff = new YoBoolean("isDesiredECMPOKForToeOff", registry);
   private final YoBoolean isDesiredCoPOKForToeOff = new YoBoolean("isDesiredCoPOKForToeOff", registry);
   private final YoBoolean isFrontFootWellPositionedForToeOff = new YoBoolean("isFrontFootWellPositionedForToeOff", registry);

   private final YoBoolean needToSwitchToToeOffForJointLimit = new YoBoolean("needToSwitchToToeOffForJointLimit", registry);
   private final YoBoolean needToSwitchToToeOffForAnkleLimit = new YoBoolean("needToSwitchToToeOffForAnkleLimit", registry);
   private final YoBoolean needToSwitchToToeOffForLeadingKneeAtLimit = new YoBoolean("needToSwitchToToeOffForLeadingKneeAtLimit", registry);
   private final YoBoolean needToSwitchToToeOffForTrailingKneeAtLimit = new YoBoolean("needToSwitchToToeOffForTrailingKneeAtLimit", registry);
   private final YoBoolean isRearAnklePitchHittingLimit = new YoBoolean("isRearAnklePitchHittingLimit", registry);
   private final YoBoolean isLeadingKneePitchHittingUpperLimit = new YoBoolean("isLeadingKneePitchHittingUpperLimit", registry);
   private final YoBoolean isRearKneePitchHittingLowerLimit = new YoBoolean("isRearKneePitchHittingLowerLimit", registry);

   private final GlitchFilteredYoBoolean isDesiredICPOKForToeOffFilt = new GlitchFilteredYoBoolean("isDesiredICPOKForToeOffFilt", registry,
                                                                                                   isDesiredICPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isCurrentICPOKForToeOffFilt = new GlitchFilteredYoBoolean("isCurrentICPOKForToeOffFilt", registry,
                                                                                                   isCurrentICPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isDesiredECMPOKForToeOffFilt = new GlitchFilteredYoBoolean("isDesiredECMPOKForToeOffFilt", registry,
                                                                                                    isDesiredECMPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isDesiredCoPOKForToeOffFilt = new GlitchFilteredYoBoolean("isDesiredCoPOKForToeOffFilt", registry,
                                                                                                   isDesiredCoPOKForToeOff, smallGlitchWindowSize);

   private final GlitchFilteredYoBoolean isRearAnklePitchHittingLimitFilt = new GlitchFilteredYoBoolean("isRearAnklePitchHittingLimitFilt", registry,
                                                                                                        isRearAnklePitchHittingLimit, largeGlitchWindowSize);
   private final GlitchFilteredYoBoolean isLeadingKneePitchHittingUpperLimitFilt = new GlitchFilteredYoBoolean("isLeadingKneePitchHittingUpperLimitFilt",
                                                                                                               registry, isLeadingKneePitchHittingUpperLimit,
                                                                                                               largeGlitchWindowSize);
   private final GlitchFilteredYoBoolean isRearKneePitchHittingLowerLimitFilt = new GlitchFilteredYoBoolean("isRearKneePitchHittingLowerLimitFilt", registry,
                                                                                                            isRearKneePitchHittingLowerLimit,
                                                                                                            largeGlitchWindowSize);

   private final DoubleProvider minStepLengthForToeOff;
   private final DoubleProvider minStepForwardForToeOff;
   private final DoubleProvider minStepHeightForToeOff;
   private final DoubleProvider extraCoMMaxHeightWithToes;

   private final YoBoolean isSideStepping = new YoBoolean("isSideStepping", registry);
   private final YoBoolean isStepLongEnough = new YoBoolean("isStepLongEnough", registry);
   private final YoBoolean isStepLongEnoughAlongX = new YoBoolean("isStepLongEnoughAlongX", registry);


   // debug variables
   private final YoDouble ecmpProximityToOnToes = new YoDouble("ecmpProximityToOnToes", registry);
   private final YoDouble copProximityToOnToes = new YoDouble("copProximityToOnToes", registry);
   private final YoDouble desiredICPProximityToOnToes = new YoDouble("desiredICPProximityToOnToes", registry);
   private final YoDouble currentICPProximityToOnToes = new YoDouble("currentICPProximityToOnToes", registry);
   private final YoDouble desiredICPProximityToLeadingFoot = new YoDouble("desiredICPProximityToLeadingFoot", registry);
   private final YoDouble currentICPProximityToLeadingFoot = new YoDouble("currentICPProximityToLeadingFoot", registry);

   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final List<FramePoint3D> contactStatePoints = new ArrayList<>();

   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final SideDependentList<FrameConvexPolygon2D> footDefaultPolygons;
   private final FrameConvexPolygon2D leadingFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D nextFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D onToesSupportPolygon = new FrameConvexPolygon2D();

   private final FramePoint2D tempLeadingFootPosition = new FramePoint2D();
   private final FramePoint2D tempTrailingFootPosition = new FramePoint2D();
   private final FramePoint3D tempLeadingFootPositionInWorld = new FramePoint3D();
   private final FramePoint3D tempTrailingFootPositionInWorld = new FramePoint3D();
   private final FrameVector2D toLeadingFoot = new FrameVector2D();

   private final YoFramePoint2D leadingFootPosition = new YoFramePoint2D("leadingFootPositionForToeOff", null, registry);

   private final HashMap<ToeContact, AbstractToeContact> toeContacts = new HashMap<>();

   private Footstep nextFootstep;

   private final FullHumanoidRobotModel fullRobotModel;
   private final ToeOffCalculator toeOffCalculator;

   private final double inPlaceWidth;

   public ToeOffManager(HighLevelHumanoidControllerToolbox controllerToolbox, ToeOffCalculator toeOffCalculator,
                        WalkingControllerParameters walkingControllerParameters, SideDependentList<? extends ContactablePlaneBody> feet,
                        YoVariableRegistry parentRegistry)
   {
      this(controllerToolbox.getFullRobotModel(), toeOffCalculator, walkingControllerParameters, feet, createFootContactStates(controllerToolbox),
           parentRegistry);
   }

   public ToeOffManager(FullHumanoidRobotModel fullRobotModel, ToeOffCalculator toeOffCalculator, WalkingControllerParameters walkingControllerParameters,
                        SideDependentList<? extends ContactablePlaneBody> feet, SideDependentList<YoPlaneContactState> footContactStates,
                        YoVariableRegistry parentRegistry)
   {
      ToeOffParameters toeOffParameters = walkingControllerParameters.getToeOffParameters();

      doToeOffIfPossibleInDoubleSupport = new BooleanParameter("doToeOffIfPossibleInDoubleSupport", registry, toeOffParameters.doToeOffIfPossible());
      doToeOffIfPossibleInSingleSupport = new BooleanParameter("doToeOffIfPossibleInSingleSupport", registry, toeOffParameters.doToeOffIfPossibleInSingleSupport());

      doToeOffWhenHittingAnkleLimit = new BooleanParameter("doToeOffWhenHittingAnkleLimit", registry, toeOffParameters.doToeOffWhenHittingAnkleLimit());
      doToeOffWhenHittingLeadingKneeUpperLimit = new BooleanParameter("doToeOffWhenHittingLeadingKneeUpperLimit", registry, toeOffParameters.doToeOffWhenHittingLeadingKneeUpperLimit());
      doToeOffWhenHittingRearKneeLowerLimit = new BooleanParameter("doToeOffWhenHittingRearKneeLowerLimit", registry, toeOffParameters.doToeOffWhenHittingTrailingKneeLowerLimit());

      ankleLowerLimitToTriggerToeOff = new DoubleParameter("ankleLowerLimitToTriggerToeOff", registry, toeOffParameters.getAnkleLowerLimitToTriggerToeOff());
      kneeUpperLimitToTriggerToeOff = new DoubleParameter("kneeUpperLimitToTriggerToeOff", registry, toeOffParameters.getKneeUpperLimitToTriggerToeOff());
      kneeLowerLimitToTriggerToeOff = new DoubleParameter("kneeLowerLimitToTriggerToeOff", registry, 0.0);
      icpPercentOfStanceForDSToeOff = new DoubleParameter("icpPercentOfStanceForDSToeOff", registry, toeOffParameters.getICPPercentOfStanceForDSToeOff());
      icpPercentOfStanceForSSToeOff = new DoubleParameter("icpPercentOfStanceForSSToeOff", registry, toeOffParameters.getICPPercentOfStanceForSSToeOff());

      icpProximityForToeOff = new DoubleParameter("icpProximityForToeOff", registry, toeOffParameters.getECMPProximityForToeOff());
      ecmpProximityForToeOff = new DoubleParameter("ecmpProximityForToeOff", registry, toeOffParameters.getECMPProximityForToeOff());
      copProximityForToeOff = new DoubleParameter("copProximityForToeOff", registry, toeOffParameters.getCoPProximityForToeOff());

      checkECMPForToeOff = new BooleanParameter("checkECMPForToeOff", registry, toeOffParameters.checkECMPLocationToTriggerToeOff());
      checkCoPForToeOff = new BooleanParameter("checkCoPForToeOff", registry, toeOffParameters.checkCoPLocationToTriggerToeOff());

      lookAtTwoStepCapturabilityForToeOff = new BooleanParameter("lookAtTwoStepCapturabilityForToeOff", registry, toeOffParameters.lookAtTwoStepCapturabilityForToeOff());

      this.toeOffCalculator = toeOffCalculator;

      this.fullRobotModel = fullRobotModel;
      this.feet = feet;

      this.inPlaceWidth = walkingControllerParameters.getSteppingParameters().getInPlaceWidth();

      double footLength = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset() + walkingControllerParameters.getSteppingParameters()
                                                                                                                                 .getFootForwardOffset();

      extraCoMMaxHeightWithToes =  new DoubleParameter("extraCoMMaxHeightWithToes", registry, extraCoMHeightWithToes);

      minStepLengthForToeOff = new DoubleParameter("minStepLengthForToeOff", registry, toeOffParameters.getMinStepLengthForToeOff());
      minStepForwardForToeOff = new DoubleParameter("minStepForwardForToeOff", registry, footLength);
      minStepHeightForToeOff = new DoubleParameter("minStepHeightForToeOff", registry, toeOffParameters.getMinStepHeightForToeOff());

      useToeLineContactInSwing = new BooleanParameter("useToeLineContactInSwing", registry, toeOffParameters.useToeOffLineContactInSwing());
      useToeLineContactInTransfer = new BooleanParameter("useToeLineContactInTransfer", registry, toeOffParameters.useToeOffLineContactInTransfer());

      updateLineContactDuringToeOff = new BooleanParameter("updateLineContactDuringToeOff", registry, toeOffParameters.updateLineContactDuringToeOff());
      updatePointContactDuringToeOff = new BooleanParameter("updatePointContactDuringToeOff", registry, toeOffParameters.updatePointContactDuringToeOff());

      footDefaultPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         footDefaultPolygons.put(robotSide, new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(feet.get(robotSide).getContactPoints2d())));
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
   public void updateToeOffStatusSingleSupport(FramePoint3D exitCMP, FramePoint2D desiredECMP, FramePoint2D desiredCoP, FramePoint2D desiredICP,
                                               FramePoint2D currentICP)
   {
      if (!doToeOffIfPossibleInSingleSupport.getValue())
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
      double percentProximity = icpPercentOfStanceForSSToeOff.getValue();

      setPolygonFromNextFootstep(nextFootSupportPolygon);

      AbstractToeContact toeContact;
      if (useToeLineContactInSwing.getValue())
      {
         computeToePointContact.set(false);
         toeContact = toeContacts.get(ToeContact.LINE);
      }
      else
      {
         computeToeLineContact.set(false);
         toeContact = toeContacts.get(ToeContact.POINT);
      }

      toeContact.updateToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, nextFootSupportPolygon);

      ReferenceFrame soleFrame = nextFootstep.getSoleReferenceFrame();
      double requiredProximity = checkICPLocations(trailingLeg, desiredICP, currentICP, toeContact.getToeOffPoint(), nextFootSupportPolygon, soleFrame,
                                                   percentProximity);
      icpProximityToLeadingFootForSSToeOff.set(requiredProximity);

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
      if (!doToeOffIfPossibleInDoubleSupport.getValue())
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

      setPolygonFromSupportFoot(trailingLeg, leadingFootSupportPolygon);
      if (lookAtTwoStepCapturabilityForToeOff.getValue() && setPolygonFromNextFootstep(nextFootSupportPolygon))
      {
         leadingFootSupportPolygon.addVertices(nextFootSupportPolygon);
         leadingFootSupportPolygon.update();
      }

      double percentProximity = icpPercentOfStanceForDSToeOff.getValue();

      AbstractToeContact toeContact;
      if (useToeLineContactInTransfer.getValue())
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
      double requiredProximity = checkICPLocations(trailingLeg, desiredICP, currentICP, toeContact.getToeOffPoint(), leadingFootSupportPolygon, soleFrame,
                                                   percentProximity);
      icpProximityToLeadingFootForDSToeOff.set(requiredProximity);

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
   private void setPolygonFromSupportFoot(RobotSide trailingLeg, FrameConvexPolygon2D polygonToPack)
   {
      RobotSide leadingLeg = trailingLeg.getOppositeSide();
      if (footContactStates != null && footContactStates.get(leadingLeg).getTotalNumberOfContactPoints() > 0)
      {
         footContactStates.get(leadingLeg).getContactFramePointsInContact(contactStatePoints);
         polygonToPack.clear(worldFrame);
         for (int i = 0; i < contactStatePoints.size(); i++)
            polygonToPack.addVertexMatchingFrame(contactStatePoints.get(i));
         polygonToPack.update();
      }
      else
      {
         polygonToPack.setIncludingFrame(footDefaultPolygons.get(leadingLeg));
         polygonToPack.changeFrameAndProjectToXYPlane(worldFrame);
      }
   }

   /**
    * Sets the predicted support polygon of the leading support foot from the next footstep.
    */
   private boolean setPolygonFromNextFootstep(FrameConvexPolygon2D polygonToPack)
   {
      if (nextFootstep == null || nextFootstep.getRobotSide() == null)
         return false;

      ReferenceFrame footstepSoleFrame = nextFootstep.getSoleReferenceFrame();
      List<Point2D> predictedContactPoints = nextFootstep.getPredictedContactPoints();
      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         polygonToPack.clear(footstepSoleFrame);
         for (int i = 0; i < predictedContactPoints.size(); i++)
            polygonToPack.addVertex(predictedContactPoints.get(i));
         polygonToPack.update();
      }
      else
      {
         ConvexPolygon2DReadOnly footPolygon = footDefaultPolygons.get(nextFootstep.getRobotSide());
         polygonToPack.setIncludingFrame(footstepSoleFrame, footPolygon);
      }
      polygonToPack.changeFrameAndProjectToXYPlane(worldFrame);

      return true;
   }

   // FIXME I think this may not be working correctly.
   private void checkECMPLocation(FramePoint2D desiredECMP)
   {
      desiredECMP.changeFrameAndProjectToXYPlane(onToesSupportPolygon.getReferenceFrame());
      ecmpProximityToOnToes.set(onToesSupportPolygon.signedDistance(desiredECMP));

      if (checkECMPForToeOff.getValue())
      {
         isDesiredECMPOKForToeOff.set(ecmpProximityToOnToes.getDoubleValue() < ecmpProximityForToeOff.getValue());
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
      desiredCoP.changeFrameAndProjectToXYPlane(onToesSupportPolygon.getReferenceFrame());
      copProximityToOnToes.set(onToesSupportPolygon.signedDistance(desiredCoP));

      if (checkCoPForToeOff.getValue())
      {
         isDesiredCoPOKForToeOff.set(copProximityToOnToes.getDoubleValue() < copProximityForToeOff.getValue());
         isDesiredCoPOKForToeOffFilt.update();
      }
      else
      {
         isDesiredCoPOKForToeOff.set(true);
         isDesiredCoPOKForToeOffFilt.set(true);
      }
   }

   private double checkICPLocations(RobotSide trailingLeg, FramePoint2D desiredICP, FramePoint2D currentICP, FramePoint2D toeOffPoint,
                                    FrameConvexPolygon2D leadingFootSupportPolygon, ReferenceFrame nextSoleFrame, double percentProximity)
   {
      desiredICPProximityToOnToes.set(onToesSupportPolygon.signedDistance(desiredICP));
      currentICPProximityToOnToes.set(onToesSupportPolygon.signedDistance(currentICP));
      desiredICPProximityToLeadingFoot.set(leadingFootSupportPolygon.signedDistance(desiredICP));
      currentICPProximityToLeadingFoot.set(leadingFootSupportPolygon.signedDistance(currentICP));

      double requiredProximityToLeadingFoot;
      if (percentProximity > 0.0)
         requiredProximityToLeadingFoot = computeRequiredICPProximity(trailingLeg, nextSoleFrame, toeOffPoint, percentProximity);
      else
         requiredProximityToLeadingFoot = 0.0;

      boolean isDesiredICPOKForToeOff = desiredICPProximityToOnToes.getDoubleValue() < icpProximityForToeOff.getValue();
      isDesiredICPOKForToeOff &= desiredICPProximityToLeadingFoot.getDoubleValue() < requiredProximityToLeadingFoot;

      boolean isCurrentICPOKForToeOff = currentICPProximityToOnToes.getDoubleValue() < icpProximityForToeOff.getValue();
      isCurrentICPOKForToeOff &= currentICPProximityToLeadingFoot.getDoubleValue() < requiredProximityToLeadingFoot;

      this.isCurrentICPOKForToeOff.set(isCurrentICPOKForToeOff);
      this.isDesiredICPOKForToeOff.set(isDesiredICPOKForToeOff);
      this.isCurrentICPOKForToeOffFilt.update();
      this.isDesiredICPOKForToeOffFilt.update();

      return requiredProximityToLeadingFoot;
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
      OneDoFJointBasics anklePitch = fullRobotModel.getLegJoint(trailingLeg, LegJointName.ANKLE_PITCH);
      double lowerLimit = Math.max(anklePitch.getJointLimitLower() + 0.02, ankleLowerLimitToTriggerToeOff.getValue()); // todo extract variable
      isRearAnklePitchHittingLimit.set(anklePitch.getQ() < lowerLimit);
      isRearAnklePitchHittingLimitFilt.update();

      if (!doToeOffWhenHittingAnkleLimit.getValue())
         return false;

      return isRearAnklePitchHittingLimitFilt.getBooleanValue();
   }

   private boolean checkLeadingKneeUpperLimitForToeOff(RobotSide leadingLeg)
   {
      OneDoFJointBasics kneePitch = fullRobotModel.getLegJoint(leadingLeg, LegJointName.KNEE_PITCH);
      double upperLimit = Math.min(kneePitch.getJointLimitUpper() - 0.02, kneeUpperLimitToTriggerToeOff.getValue()); // todo extract variable
      isLeadingKneePitchHittingUpperLimit.set(kneePitch.getQ() > upperLimit);
      isLeadingKneePitchHittingUpperLimitFilt.update();

      if (!doToeOffWhenHittingLeadingKneeUpperLimit.getValue())
         return false;

      return isLeadingKneePitchHittingUpperLimitFilt.getBooleanValue();
   }

   private boolean checkRearKneeLowerLimitForToeOff(RobotSide trailingLeg)
   {
      OneDoFJointBasics kneePitch = fullRobotModel.getLegJoint(trailingLeg, LegJointName.KNEE_PITCH);
      double lowerLimit = Math.max(kneePitch.getJointLimitLower() + 0.02, kneeLowerLimitToTriggerToeOff.getValue()); // todo extract variable
      isRearKneePitchHittingLowerLimit.set(kneePitch.getQ() < lowerLimit);
      isRearKneePitchHittingLowerLimitFilt.update();

      if (!doToeOffWhenHittingRearKneeLowerLimit.getValue())
         return false;

      return isRearKneePitchHittingLowerLimitFilt.getBooleanValue();
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

      leadingFootPosition.set(tempLeadingFootPosition.getX(), tempLeadingFootPosition.getY());

      tempLeadingFootPositionInWorld.setToZero(frontFootFrame);
      tempTrailingFootPositionInWorld.setToZero(trailingFootFrame);
      tempLeadingFootPositionInWorld.changeFrame(worldFrame);
      tempTrailingFootPositionInWorld.changeFrame(worldFrame);

      double stepHeight = tempLeadingFootPositionInWorld.getZ() - tempTrailingFootPositionInWorld.getZ();

      boolean isNextStepHighEnough = stepHeight > minStepHeightForToeOff.getValue();
      if (isNextStepHighEnough)
         return true;

      boolean isForwardStepping = tempLeadingFootPosition.getX() > forwardSteppingThreshold;
      if (!isForwardStepping)
         return false;

      if (ENABLE_TOE_OFF_FOR_STEP_DOWN)
      {
         boolean isNextStepLowEnough = stepHeight < -minStepHeightForToeOff.getValue();
         if (isNextStepLowEnough)
            return true;
      }
      else
      {
         boolean isNextStepTooLow = stepHeight < stepDownTooFarForToeOff;
         if (isNextStepTooLow)
            return false;
      }

      isSideStepping.set(Math.abs(Math.atan2(tempLeadingFootPosition.getY(), tempLeadingFootPosition.getX())) > Math.toRadians(minimumAngleForSideStepping));
      if (isSideStepping.getValue() && !DO_TOE_OFF_FOR_SIDE_STEPS)
         return false;

      isStepLongEnough.set(tempLeadingFootPosition.distance(tempTrailingFootPosition) > minStepLengthForToeOff.getValue());
      isStepLongEnoughAlongX.set(tempLeadingFootPosition.getX() > minStepForwardForToeOff.getValue());
      return isStepLongEnough.getValue() && isStepLongEnoughAlongX.getValue();
   }

   /**
    * Checks whether or not the next footstep in {@param nextFootstep} is in correct location to achieve toe off.
    * @param nextFootstep footstep to consider.
    * @param transferToSide upcoming support side.
    * @return whether or not the footstep location is ok.
    */
   public boolean canDoSingleSupportToeOff(Footstep nextFootstep, RobotSide transferToSide)
   {
      if (!doToeOffIfPossibleInSingleSupport.getValue())
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
      if (!doToeOffIfPossibleInDoubleSupport.getValue())
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
      return extraCoMMaxHeightWithToes.getValue();
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

      public abstract void updateToeSupportPolygon(FramePoint3D exitCMP, FramePoint2D desiredECMP, RobotSide trailingSide,
                                                   FrameConvexPolygon2DReadOnly leadingSupportPolygon);

      public abstract void isReadyToSwitchToToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame);

      public abstract boolean evaluateToeOffConditions(RobotSide trailingLeg);

      protected void computeToeContacts(RobotSide supportSide)
      {
         FrameConvexPolygon2D footDefaultPolygon = footDefaultPolygons.get(supportSide);
         ReferenceFrame referenceFrame = footDefaultPolygon.getReferenceFrame();
         toeOffLine.setFirstEndpoint(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);
         toeOffLine.setSecondEndpoint(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);

         // gets the leading two toe points
         for (int i = 0; i < footDefaultPolygon.getNumberOfVertices(); i++)
         {
            tmpPoint2d.setIncludingFrame(footDefaultPolygon.getVertex(i));
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
      public void updateToeSupportPolygon(FramePoint3D exitCMP, FramePoint2D desiredECMP, RobotSide trailingSide,
                                          FrameConvexPolygon2DReadOnly leadingSupportPolygon)
      {
         if (exitCMP == null)
            computeToeContacts(trailingSide);
         else
            computeToeContacts(exitCMP, desiredECMP, trailingSide);

         onToesSupportPolygon.setIncludingFrame(leadingSupportPolygon);
         onToesSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

         onToesSupportPolygon.addVertexMatchingFrame(toeOffLine.getFirstEndpoint(), false);
         onToesSupportPolygon.addVertexMatchingFrame(toeOffLine.getSecondEndpoint(), false);

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

         computeToeLineContact.set(updateLineContactDuringToeOff.getValue());
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
         boolean ankleAtLimit = checkAnkleLimitForToeOff(trailingLeg);
         boolean leadingKneeAtLimit = checkLeadingKneeUpperLimitForToeOff(trailingLeg.getOppositeSide());
         boolean trailingKneeAtLimit = checkRearKneeLowerLimitForToeOff(trailingLeg);

         needToSwitchToToeOffForAnkleLimit.set(ankleAtLimit);
         needToSwitchToToeOffForLeadingKneeAtLimit.set(leadingKneeAtLimit);
         needToSwitchToToeOffForTrailingKneeAtLimit.set(trailingKneeAtLimit);

         if (!isDesiredICPOKForToeOffFilt.getBooleanValue())
         {
            doLineToeOff.set(false);
            computeToeLineContact.set(true);
            return false;
         }

         needToSwitchToToeOffForJointLimit.set(ankleAtLimit || leadingKneeAtLimit || trailingKneeAtLimit);
         if (needToSwitchToToeOffForJointLimit.getBooleanValue())
         {
            doLineToeOff.set(true);
            computeToeLineContact.set(updateLineContactDuringToeOff.getValue());
            return false;
         }

         if (!isCurrentICPOKForToeOffFilt.getBooleanValue())
         {
            doLineToeOff.set(false);
            computeToeLineContact.set(true);
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
      public void updateToeSupportPolygon(FramePoint3D exitCMP, FramePoint2D desiredECMP, RobotSide trailingSide,
                                          FrameConvexPolygon2DReadOnly leadingSupportPolygon)
      {
         if (exitCMP == null)
            computeToeContacts(trailingSide);
         else
            computeToeContacts(exitCMP, desiredECMP, trailingSide);

         onToesSupportPolygon.setIncludingFrame(leadingSupportPolygon);
         onToesSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

         onToesSupportPolygon.addVertexMatchingFrame(toeOffPoint, false);
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

         computeToePointContact.set(updatePointContactDuringToeOff.getValue());
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
         boolean ankleAtLimit = checkAnkleLimitForToeOff(trailingLeg);
         boolean leadingKneeAtLimit = checkLeadingKneeUpperLimitForToeOff(trailingLeg.getOppositeSide());
         boolean trailingKneeAtLimit = checkRearKneeLowerLimitForToeOff(trailingLeg);

         needToSwitchToToeOffForAnkleLimit.set(ankleAtLimit);
         needToSwitchToToeOffForLeadingKneeAtLimit.set(leadingKneeAtLimit);
         needToSwitchToToeOffForTrailingKneeAtLimit.set(trailingKneeAtLimit);

         if (!isDesiredICPOKForToeOffFilt.getBooleanValue())
         {
            doLineToeOff.set(false);
            computeToeLineContact.set(true);
            return false;
         }

         needToSwitchToToeOffForJointLimit.set(ankleAtLimit || leadingKneeAtLimit || trailingKneeAtLimit);
         if (needToSwitchToToeOffForJointLimit.getBooleanValue())
         {
            doPointToeOff.set(true);
            computeToePointContact.set(updatePointContactDuringToeOff.getValue());
            return false;
         }

         if (!isCurrentICPOKForToeOffFilt.getBooleanValue())
         {
            doLineToeOff.set(false);
            computeToeLineContact.set(true);
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