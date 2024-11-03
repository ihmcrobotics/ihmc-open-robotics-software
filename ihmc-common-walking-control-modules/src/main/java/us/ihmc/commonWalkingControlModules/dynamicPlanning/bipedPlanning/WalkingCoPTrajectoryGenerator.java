package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import java.util.List;
import java.util.function.Supplier;

import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.DefaultSplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionFromAreaCalculator;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionFromPositionCalculator;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.YoSplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingCoPTrajectoryGenerator extends CoPTrajectoryGenerator implements SCS2YoGraphicHolder
{
   private final CoPTrajectoryParameters parameters;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private final SideDependentList<ConvexPolygon2D> defaultSupportPolygons = new SideDependentList<>();
   private final SideDependentList<FrameConvexPolygon2D> movingPolygonsInSole = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final SideDependentList<RecyclingArrayList<PoseReferenceFrame>> stepFrames = new SideDependentList<>();

   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final FramePose3D tempPose = new FramePose3D();

   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final FramePoint3D tempFramePoint1 = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final FramePoint3D tempPointForCoPCalculation = new FramePoint3D();
   private final FramePoint3D previousCoPPosition = new FramePoint3D();
   private final FramePoint3D midfootCoP = new FramePoint3D();
   private final FramePoint2D copInFootFrame = new FramePoint2D();

   private final YoDouble finalTransferSplitFraction;
   private final YoDouble finalTransferWeightDistribution;
   private final PreallocatedList<YoDouble> transferSplitFractions;
   private final PreallocatedList<YoDouble> transferWeightDistributions;

   private final SplitFractionFromPositionCalculator positionSplitFractionCalculator;
   private final SplitFractionFromAreaCalculator areaSplitFractionCalculator;

   private final YoSplitFractionCalculatorParameters splitFractionParameters;

   private int shiftFractionCounter = 0;
   private int weightDistributionCounter = 0;

   private boolean holdSplitFractionParameters = false;

   private CoPPointViewer viewer = null;

   public WalkingCoPTrajectoryGenerator(CoPTrajectoryParameters parameters,
                                        SideDependentList<? extends ConvexPolygon2DReadOnly> defaultSupportPolygons,
                                        YoRegistry parentRegistry)
   {
      this(parameters, new DefaultSplitFractionCalculatorParameters(), defaultSupportPolygons, parentRegistry);
   }

   public WalkingCoPTrajectoryGenerator(CoPTrajectoryParameters parameters,
                                        SplitFractionCalculatorParametersReadOnly defaultSplitFractionParameters,
                                        SideDependentList<? extends ConvexPolygon2DReadOnly> defaultSupportPolygons,
                                        YoRegistry parentRegistry)
   {
      super(WalkingCoPTrajectoryGenerator.class, parentRegistry);

      this.parameters = parameters;
      this.defaultSupportPolygons.set(side -> new ConvexPolygon2D(defaultSupportPolygons.get(side)));

      registry = new YoRegistry(getClass().getSimpleName());
      splitFractionParameters = new YoSplitFractionCalculatorParameters(defaultSplitFractionParameters, registry);
      for (RobotSide robotSide : RobotSide.values)
      {
         stepFrames.put(robotSide, new RecyclingArrayList<>(3, new Supplier<PoseReferenceFrame>()
         {
            private int frameIndexCounter = 0;

            @Override
            public PoseReferenceFrame get()
            {
               return new PoseReferenceFrame(robotSide.getLowerCaseName() + "StepFrame" + frameIndexCounter++, ReferenceFrame.getWorldFrame());
            }
         }));
      }

      finalTransferWeightDistribution = new YoDouble("processedFinalTransferWeightDistribution", registry);
      finalTransferSplitFraction = new YoDouble("processedFinalTransferSplitFraction", registry);

      transferSplitFractions = new PreallocatedList<>(YoDouble.class, () -> new YoDouble("processedTransferSplitFraction" + shiftFractionCounter++, registry), parameters.getMaxNumberOfStepsToConsider());
      transferWeightDistributions = new PreallocatedList<>(YoDouble.class, () -> new YoDouble("processedTransferWeightDistribution" + weightDistributionCounter++, registry), parameters.getMaxNumberOfStepsToConsider());

      positionSplitFractionCalculator = new SplitFractionFromPositionCalculator(splitFractionParameters);

      areaSplitFractionCalculator = new SplitFractionFromAreaCalculator(splitFractionParameters, defaultSupportPolygons);

      parentRegistry.addChild(registry);
      clear();
   }

   @Override
   public void registerState(CoPTrajectoryGeneratorState state)
   {
      super.registerState(state);

      state.registerStateToSave(splitFractionParameters);

      positionSplitFractionCalculator.setNumberOfStepsProvider(state::getNumberOfFootstep);

      positionSplitFractionCalculator.setFinalTransferSplitFractionProvider(finalTransferSplitFraction::getDoubleValue);
      positionSplitFractionCalculator.setFinalTransferWeightDistributionProvider(finalTransferWeightDistribution::getDoubleValue);

      positionSplitFractionCalculator.setTransferSplitFractionProvider((i) -> transferSplitFractions.get(i).getDoubleValue());
      positionSplitFractionCalculator.setTransferWeightDistributionProvider((i) -> transferWeightDistributions.get(i).getDoubleValue());

      positionSplitFractionCalculator.setFinalTransferSplitFractionConsumer(finalTransferSplitFraction::set);
      positionSplitFractionCalculator.setFinalTransferWeightDistributionConsumer(finalTransferWeightDistribution::set);

      positionSplitFractionCalculator.setTransferWeightDistributionConsumer((i, d) -> transferWeightDistributions.get(i).set(d));
      positionSplitFractionCalculator.setTransferSplitFractionConsumer((i, d) -> transferSplitFractions.get(i).set(d));

      positionSplitFractionCalculator.setFirstSupportPoseProvider(() ->
                                                                  {
                                                                     RobotSide stanceSide = state.getFootstep(0).getRobotSide().getOppositeSide();
                                                                     return state.getCurrentFootPose(stanceSide);
                                                                  });
      positionSplitFractionCalculator.setFirstSwingPoseProvider(() ->
                                                                {
                                                                   RobotSide stanceSide = state.getFootstep(0).getRobotSide();
                                                                   return state.getCurrentFootPose(stanceSide);
                                                                });
      positionSplitFractionCalculator.setStepPoseGetter((i) -> state.getFootstep(i).getFootstepPose());

      areaSplitFractionCalculator.setNumberOfStepsProvider(state::getNumberOfFootstep);

      areaSplitFractionCalculator.setFinalTransferSplitFractionProvider(finalTransferSplitFraction::getDoubleValue);
      areaSplitFractionCalculator.setFinalTransferWeightDistributionProvider(finalTransferWeightDistribution::getDoubleValue);

      areaSplitFractionCalculator.setTransferSplitFractionProvider((i) -> transferSplitFractions.get(i).getDoubleValue());
      areaSplitFractionCalculator.setTransferWeightDistributionProvider((i) -> transferWeightDistributions.get(i).getDoubleValue());

      areaSplitFractionCalculator.setFinalTransferSplitFractionConsumer(finalTransferSplitFraction::set);
      areaSplitFractionCalculator.setFinalTransferWeightDistributionConsumer(finalTransferWeightDistribution::set);

      areaSplitFractionCalculator.setTransferWeightDistributionConsumer((i, d) -> transferWeightDistributions.get(i).set(d));
      areaSplitFractionCalculator.setTransferSplitFractionConsumer((i, d) -> transferSplitFractions.get(i).set(d));

      areaSplitFractionCalculator.setFirstSupportPolygonProvider(() ->
                                                                 {
                                                                    RobotSide stanceSide = state.getFootstep(0).getRobotSide().getOppositeSide();
                                                                    return state.getFootPolygonInSole(stanceSide).getPolygonVerticesView();
                                                                 });
      areaSplitFractionCalculator.setStepSideProvider((i) -> state.getFootstep(i).getRobotSide());
      areaSplitFractionCalculator.setStepPolygonGetter((i) -> state.getFootstep(i).getPredictedContactPoints());
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public void setWaypointViewer(CoPPointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void clear()
   {
      contactStateProviders.clear();
   }

   private void reset(CoPTrajectoryGeneratorState state)
   {
      contactStateProviders.clear();
      for (RobotSide robotSide : RobotSide.values)
         stepFrames.get(robotSide).clear();

      if(!holdSplitFractionParameters)
      {
         finalTransferSplitFraction.set(parameters.getDefaultFinalTransferSplitFraction());
         finalTransferWeightDistribution.set(parameters.getDefaultFinalTransferWeightDistribution());
         for (int i = 0; i < transferSplitFractions.size(); i++)
         {
            transferSplitFractions.get(i).setToNaN();
            transferWeightDistributions.get(i).setToNaN();
         }
         transferSplitFractions.clear();
         transferWeightDistributions.clear();
         for (int i = 0; i < state.getNumberOfFootstep(); i++)
         {
            transferSplitFractions.add().set(parameters.getDefaultTransferSplitFraction());
            transferWeightDistributions.add().set(parameters.getDefaultTransferWeightDistribution());
         }
      }
   }

   public void setHoldSplitFractions(boolean hold)
   {
      holdSplitFractionParameters = hold;
   }

   private final FrameConvexPolygon2DBasics nextPolygon = new FrameConvexPolygon2D();

   private final FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private final FramePoint2D initialCoP = new FramePoint2D();
   private final FramePoint3D restrictedInitialCoP = new FramePoint3D();

   public void compute(CoPTrajectoryGeneratorState state)
   {
      int numberOfUpcomingFootsteps = Math.min(parameters.getNumberOfStepsToConsider(), state.getNumberOfFootstep());

      reset(state);
      supportPolygon.clear(worldFrame);

      // Add initial support states of the feet and set the moving polygons
      for (RobotSide robotSide : RobotSide.values)
      {
         // Record the initial step frames. In case there is a step touching down this frame will be updated.
         PoseReferenceFrame stepFrame = stepFrames.get(robotSide).add();
         tempPose.setIncludingFrame(state.getCurrentFootPose(robotSide));
         tempPose.changeFrame(stepFrame.getParent());
         stepFrame.setPoseAndUpdate(tempPose);

         movingPolygonsInSole.get(robotSide).setIncludingFrame(state.getFootPolygonInSole(robotSide));
         movingPolygonsInSole.get(robotSide).changeFrameAndProjectToXYPlane(stepFrame);
         supportPolygon.addVerticesMatchingFrame(movingPolygonsInSole.get(robotSide), false);
      }
      supportPolygon.update();

      positionSplitFractionCalculator.computeSplitFractionsFromPosition();
      areaSplitFractionCalculator.computeSplitFractionsFromArea();

      initialCoP.set(state.getInitialCoP());
      if (!supportPolygon.isPointInside(initialCoP))
         supportPolygon.orthogonalProjection(initialCoP);
      restrictedInitialCoP.set(initialCoP, state.getInitialCoP().getZ());

      // compute cop waypoint location
      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartTime(0.0);
      contactStateProvider.setStartECMPPosition(restrictedInitialCoP);

      // Put first CoP as per chicken support computations in case starting from rest
      if (numberOfUpcomingFootsteps == 0)
      {
         // just standing there
         computeCoPPointsForStanding(state);
      }
      else
      {
         // Compute all the upcoming waypoints
         for (int footstepIndex = 0; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
         {
            DynamicPlanningFootstep footstep = state.getFootstep(footstepIndex);
            PlanningTiming timings = state.getTiming(footstepIndex);
            RobotSide swingSide = footstep.getRobotSide();
            RobotSide supportSide = swingSide.getOppositeSide();

            FrameConvexPolygon2DReadOnly previousPolygon = movingPolygonsInSole.get(swingSide);
            FrameConvexPolygon2DReadOnly currentPolygon = movingPolygonsInSole.get(swingSide.getOppositeSide());

            ReferenceFrame stepFrame = extractStepFrame(footstep);
            extractSupportPolygon(footstep, stepFrame, nextPolygon, defaultSupportPolygons.get(footstep.getRobotSide()));

            computeCoPPointsForFootstepTransfer(timings.getTransferTime(),
                                                transferSplitFractions.get(footstepIndex).getDoubleValue(),
                                                transferWeightDistributions.get(footstepIndex).getDoubleValue(),
                                                previousPolygon,
                                                currentPolygon,
                                                supportSide,
                                                footstepIndex == 0);
            computeCoPPointsForFootstepSwing(Math.min(timings.getSwingTime(), CoMTrajectoryPlannerTools.sufficientlyLongTime),
                                             parameters.getDefaultSwingDurationShiftFraction(),
                                             parameters.getDefaultSwingSplitFraction(),
                                             currentPolygon,
                                             nextPolygon,
                                             supportSide);

            movingPolygonsInSole.get(swingSide).setIncludingFrame(nextPolygon);
         }
         computeCoPPointsForFinalTransfer(state.getFootstep(numberOfUpcomingFootsteps - 1).getRobotSide().getOppositeSide(),
                                          state.getFinalTransferDuration(),
                                          finalTransferWeightDistribution.getDoubleValue());

         RobotSide lastStepSide = state.getFootstep(numberOfUpcomingFootsteps - 1).getRobotSide().getOppositeSide();
         if (lastStepSide == RobotSide.RIGHT)
            state.setPercentageStandingWeightDistributionOnLeftFoot(1.0 - finalTransferWeightDistribution.getDoubleValue());
         else
            state.setPercentageStandingWeightDistributionOnLeftFoot(finalTransferWeightDistribution.getDoubleValue());
      }

      if (viewer != null)
         viewer.updateWaypoints(contactStateProviders);
   }


   public void update(double time, FixedFramePoint3DBasics desiredCoP)
   {
      for (int i = 0; i < contactStateProviders.size(); i++)
      {
         ContactStateProvider contactStateProvider = contactStateProviders.get(i);
         if (contactStateProvider.getTimeInterval().intervalContains(time))
         {
            double alpha = (time - contactStateProvider.getTimeInterval().getStartTime()) / contactStateProvider.getTimeInterval().getDuration();
            desiredCoP.interpolate(contactStateProvider.getECMPStartPosition(), contactStateProvider.getECMPEndPosition(), alpha);
            break;
         }
         else
         {
            continue;
         }
      }
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }

   private ReferenceFrame extractStepFrame(DynamicPlanningFootstep footstep)
   {
      PoseReferenceFrame stepFrame = stepFrames.get(footstep.getRobotSide()).add();
      stepFrame.setPoseAndUpdate(footstep.getFootstepPose());

      return stepFrame;
   }

   private void extractSupportPolygon(DynamicPlanningFootstep footstep,
                                      ReferenceFrame stepFrame,
                                      FrameConvexPolygon2DBasics footSupportPolygonToPack,
                                      ConvexPolygon2DReadOnly defaultSupportPolygon)
   {
      if (footstep.hasPredictedContactPoints())
      {
         List<? extends Point2DReadOnly> predictedContactPoints = footstep.getPredictedContactPoints();

         footSupportPolygonToPack.clear(stepFrame);
         for (int i = 0; i < predictedContactPoints.size(); i++)
            footSupportPolygonToPack.addVertex(predictedContactPoints.get(i));
         footSupportPolygonToPack.update();
      }
      else
      {
         footSupportPolygonToPack.setIncludingFrame(stepFrame, defaultSupportPolygon);
      }
   }


   private void computeCoPPointsForStanding(CoPTrajectoryGeneratorState state)
   {
      computeCoPPointsForFinalTransfer(RobotSide.LEFT, state.getFinalTransferDuration(), state.getPercentageStandingWeightDistributionOnLeftFoot());

      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndECMPPosition(previousContactState.getECMPStartPosition());
      contactState.setDuration(Double.POSITIVE_INFINITY);
      contactState.setLinearECMPVelocity();
   }

   private void computeCoPPointsForFinalTransfer(RobotSide lastFootstepSide,
                                                 double finalTransferDuration,
                                                 double finalTransferWeightDistribution)
   {
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();

      tempPointForCoPCalculation.setIncludingFrame(movingPolygonsInSole.get(lastFootstepSide).getCentroid(), 0.0);
      tempPointForCoPCalculation.changeFrame(worldFrame);
      tempFramePoint1.setIncludingFrame(movingPolygonsInSole.get(lastFootstepSide.getOppositeSide()).getCentroid(), 0.0);
      tempFramePoint1.changeFrame(worldFrame);
      tempPointForCoPCalculation.interpolate(tempFramePoint1, finalTransferWeightDistribution);

      double segmentDuration = finalTransferSplitFraction.getDoubleValue() * finalTransferDuration;
      previousContactState.setEndECMPPosition(tempPointForCoPCalculation);
      previousContactState.setDuration(segmentDuration);
      previousContactState.setLinearECMPVelocity();

      segmentDuration = (1.0 - finalTransferSplitFraction.getValue()) * finalTransferDuration;
      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndECMPPosition(tempPointForCoPCalculation);
      contactState.setDuration(segmentDuration);
      contactState.setLinearECMPVelocity();
   }


   private void computeCoPPointsForFootstepTransfer(double duration,
                                                    double splitFraction,
                                                    double weightDistribution,
                                                    FrameConvexPolygon2DReadOnly previousPolygon,
                                                    FrameConvexPolygon2DReadOnly nextPolygon,
                                                    RobotSide supportSide,
                                                    boolean setupForContinuityMaintaining)
   {
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      previousCoPPosition.setIncludingFrame(previousContactState.getECMPStartPosition());
      previousCoPPosition.changeFrame(previousPolygon.getReferenceFrame());

      if (!previousPolygon.isPointInside(previousCoPPosition.getX(), previousCoPPosition.getY()))
      {
         computeExitCoPLocation(previousCoPPosition, previousPolygon, nextPolygon, supportSide.getOppositeSide());
      }

      previousCoPPosition.changeFrame(worldFrame);
      computeEntryCoPPointLocation(tempPointForCoPCalculation, previousPolygon, nextPolygon, supportSide);
      midfootCoP.interpolate(previousCoPPosition, tempPointForCoPCalculation, weightDistribution);

      double firstSegmentDuration = splitFraction * duration;
      if (setupForContinuityMaintaining)
         firstSegmentDuration = Math.min(firstSegmentDuration, parameters.getDurationForContinuityMaintenanceSegment());
      previousContactState.setDuration(firstSegmentDuration);
      previousContactState.setEndECMPPosition(midfootCoP);
      previousContactState.setLinearECMPVelocity();

      double secondSegmentDuration = duration - firstSegmentDuration;
      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartFromEnd(previousContactState);
      contactStateProvider.setDuration(secondSegmentDuration);
      contactStateProvider.setEndECMPPosition(tempPointForCoPCalculation);
      contactStateProvider.setLinearECMPVelocity();
   }

   private void computeCoPPointsForFootstepSwing(double duration,
                                                 double shiftFraction,
                                                 double splitFraction,
                                                 FrameConvexPolygon2DReadOnly supportPolygon,
                                                 FrameConvexPolygon2DReadOnly nextPolygon,
                                                 RobotSide supportSide)
   {
      computeBallCoPLocation(tempPointForCoPCalculation, supportPolygon, nextPolygon, supportSide);
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setDuration(shiftFraction * splitFraction * duration);
      contactState.setEndECMPPosition(tempPointForCoPCalculation);
      contactState.setLinearECMPVelocity();

      previousContactState = contactState;

      computeExitCoPLocation(tempPointForCoPCalculation, supportPolygon, nextPolygon, supportSide);

      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setDuration(shiftFraction * (1.0 - splitFraction) * duration);
      contactState.setEndECMPPosition(tempPointForCoPCalculation);
      contactState.setLinearECMPVelocity();

      previousContactState = contactState;
      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setDuration((1.0 - shiftFraction) * duration);
      contactState.setEndECMPPosition(tempPointForCoPCalculation);
      contactState.setLinearECMPVelocity();

      contactStateProviders.add().setStartFromEnd(contactState);
   }

   private void computeEntryCoPPointLocation(FramePoint3DBasics copLocationToPack,
                                             FrameConvexPolygon2DReadOnly previousFootPolygon,
                                             FrameConvexPolygon2DReadOnly footPolygon,
                                             RobotSide supportSide)
   {
      computeCoPLocation(copLocationToPack,
                         parameters.getEntryCMPLengthOffsetFactor(),
                         parameters.getEntryCMPOffset(),
                         parameters.getEntryCMPMinX(),
                         parameters.getEntryCMPMaxX(),
                         footPolygon,
                         previousFootPolygon,
                         supportSide);
   }

   private void computeBallCoPLocation(FramePoint3DBasics copLocationToPack,
                                       FrameConvexPolygon2DReadOnly footPolygon,
                                       FrameConvexPolygon2DReadOnly nextFootPolygon,
                                       RobotSide supportSide)
   {
      computeCoPLocation(copLocationToPack,
                         parameters.getBallCMPLengthOffsetFactor(),
                         parameters.getBallCMPOffset(),
                         parameters.getBallCMPMinX(),
                         parameters.getBallCMPMaxX(),
                         footPolygon,
                         nextFootPolygon,
                         supportSide);
   }

   private void computeExitCoPLocation(FramePoint3DBasics copLocationToPack,
                                       FrameConvexPolygon2DReadOnly footPolygon,
                                       FrameConvexPolygon2DReadOnly nextFootPolygon,
                                       RobotSide supportSide)
   {
      tempFramePoint1.setIncludingFrame(nextFootPolygon.getCentroid(), 0.0);
      tempFramePoint1.changeFrame(footPolygon.getReferenceFrame());
      tempFramePoint2.setIncludingFrame(footPolygon.getCentroid(), 0.0);
      double supportToSwingStepLength = tempFramePoint1.getX() - tempFramePoint2.getX();
      double supportToSwingStepHeight = tempFramePoint1.getZ() - tempFramePoint2.getZ();

      if (setExitCoPIfSupportIsEmpty(copLocationToPack, footPolygon, supportSide))
         return;

      double lengthOffsetFactor = parameters.getExitCMPLengthOffsetFactor();
      if (supportToSwingStepHeight < parameters.getStepHeightToPutExitCoPOnToesSteppingDown())
         lengthOffsetFactor *= parameters.getStepDownLengthOffsetScaleFactor();

      computeCoPLocation(copLocationToPack,
                         lengthOffsetFactor,
                         parameters.getExitCMPOffset(),
                         parameters.getExitCMPMinX(),
                         parameters.getExitCMPMaxX(),
                         footPolygon,
                         nextFootPolygon,
                         supportSide);

      setExitCoPUnderSpecialCases(copLocationToPack, footPolygon, supportSide, supportToSwingStepHeight);
   }

   private void computeCoPLocation(FramePoint3DBasics copLocationToPack,
                                   double lengthOffsetFactor,
                                   Vector2DReadOnly copOffset,
                                   double minXOffset,
                                   double maxXOffset,
                                   FrameConvexPolygon2DReadOnly basePolygon,
                                   FrameConvexPolygon2DReadOnly otherPolygon,
                                   RobotSide supportSide)
   {
      // FIXME this should be done in the sole frame, not the world frame
      copInFootFrame.setIncludingFrame(basePolygon.getCentroid());



      double copXOffset = MathTools.clamp(copOffset.getX() + lengthOffsetFactor * getStepLength(otherPolygon, basePolygon),
                                          minXOffset,
                                          maxXOffset);
      copInFootFrame.add(copXOffset, supportSide.negateIfLeftSide(copOffset.getY()));

      constrainToPolygon(copInFootFrame, basePolygon, parameters.getMinimumDistanceInsidePolygon());

      copLocationToPack.setIncludingFrame(copInFootFrame, 0.0);
      copLocationToPack.changeFrame(worldFrame);
   }

   /**
    * Checks for the following conditions to have been the case:
    * - the support polygon is empty
    * - the exit CoP goes in the toes
    * - the exit CoP goes in the toes when stepping down
    *
    * @return true if any of these cases held, at which point the CoP has been placed. False if none of them held, at which it still needs to be computed.
    */
   private boolean setExitCoPIfSupportIsEmpty(FramePoint3DBasics framePointToPack,
                                               FrameConvexPolygon2DReadOnly supportFootPolygon,
                                               RobotSide supportSide)
   {

      if (supportFootPolygon.getArea() == 0.0)
      { // FIXME this is bad if it's a line, right?
         if (supportFootPolygon.getNumberOfVertices() == 2)
         {
            copInFootFrame.setToZero(supportFootPolygon.getReferenceFrame());
            copInFootFrame.interpolate(supportFootPolygon.getVertex(0), supportFootPolygon.getVertex(1), 0.5);
            copInFootFrame.addY(supportSide.negateIfLeftSide(parameters.getExitCMPOffset().getY()));
            if (!supportFootPolygon.isPointInside(copInFootFrame))
               supportFootPolygon.orthogonalProjection(copInFootFrame);
         }
         else
         {
            copInFootFrame.setIncludingFrame(supportFootPolygon.getReferenceFrame(), supportFootPolygon.getVertex(0));
         }
         copInFootFrame.changeFrameAndProjectToXYPlane(worldFrame);

         framePointToPack.setIncludingFrame(copInFootFrame, 0.0);
         framePointToPack.changeFrame(worldFrame);
         return true;
      }
      return false;
   }

   /**
    * Checks for the following conditions to have been the case:
    * - the support polygon is empty
    * - the exit CoP goes in the toes
    * - the exit CoP goes in the toes when stepping down
    *
    * @return true if any of these cases held, at which point the CoP has been placed. False if none of them held, at which it still needs to be computed.
    */
   private void setExitCoPUnderSpecialCases(FramePoint3DBasics framePointToPack,
                                            FrameConvexPolygon2DReadOnly supportFootPolygon,
                                            RobotSide supportSide,
                                            double stepHeight)
   {

      framePointToPack.changeFrame(supportFootPolygon.getReferenceFrame());

      if (parameters.getPlanForToeOffCalculator().shouldPutCMPOnToes(stepHeight, framePointToPack, supportFootPolygon))
      {
         copInFootFrame.changeFrame(supportFootPolygon.getReferenceFrame());

         copInFootFrame.setIncludingFrame(supportFootPolygon.getCentroid());
         copInFootFrame.add(supportFootPolygon.getMaxX() - parameters.getExitCoPForwardSafetyMarginOnToes(),
                              supportSide.negateIfLeftSide(parameters.getExitCMPOffset().getY()));

         constrainToPolygon(copInFootFrame, supportFootPolygon, parameters.getExitCoPForwardSafetyMarginOnToes());

         framePointToPack.setIncludingFrame(copInFootFrame, 0.0);
      }
      framePointToPack.changeFrame(worldFrame);
   }

   private final FramePoint2D mostForwardPointOnOtherPolygon = new FramePoint2D();

   private double getStepLength(FrameConvexPolygon2DReadOnly otherPolygon, FrameConvexPolygon2DReadOnly basePolygon)
   {
      mostForwardPointOnOtherPolygon.setIncludingFrame(otherPolygon.getVertex(EuclidGeometryPolygonTools.findVertexIndex(otherPolygon,
                                                                                                                         true,
                                                                                                                         Bound.MAX,
                                                                                                                         Bound.MAX)));
      mostForwardPointOnOtherPolygon.changeFrameAndProjectToXYPlane(basePolygon.getReferenceFrame());

     return mostForwardPointOnOtherPolygon.getX() - basePolygon.getMaxX();
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by
    * projection
    */
   private void constrainToPolygon(FramePoint2DBasics copPointToConstrain,
                                   FrameConvexPolygon2DReadOnly constraintPolygon,
                                   double safeDistanceFromSupportPolygonEdges)
   {
      // TODO this is not very efficient
      // don't need to do anything if it's already inside
      if (constraintPolygon.signedDistance(copPointToConstrain) <= -safeDistanceFromSupportPolygonEdges)
         return;

      polygonScaler.scaleConvexPolygon(constraintPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      copPointToConstrain.changeFrame(constraintPolygon.getReferenceFrame());
      if (!tempPolygon.isPointInside(copPointToConstrain))
         tempPolygon.orthogonalProjection(copPointToConstrain);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return viewer == null ? null : viewer.getSCS2YoGraphics();
   }
}