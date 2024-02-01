package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public class MultiStepPushRecoveryControlModule implements SCS2YoGraphicHolder
{
   private static final boolean ENABLE_SQUARE_UP = false;

   private final GlitchFilteredYoBoolean isRobotBackToSafeState;
   private final YoBoolean isICPOutside;
   private final YoEnum<RobotSide> swingSideForDoubleSupportRecovery;

   private final SideDependentList<MultiStepPushRecoveryCalculator> pushRecoveryCalculators = new SideDependentList<>();
   private final MultiStepPushRecoveryCalculatorVisualizer pushRecoveryCalculatorVisualizer;

   private final SideDependentList<YoPlaneContactState> contactStates;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private final  DoubleProvider pushRecoveryTransferDuration;
   private final  DoubleProvider pushRecoveryPreferredSwingDuration;
   private final  DoubleProvider pushRecoveryMinSwingDuration;
   private final  DoubleProvider pushRecoveryMaxSwingDuration;

   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();
   private final RecyclingArrayList<Footstep> recoveryFootsteps = new RecyclingArrayList<>(Footstep::new);
   private final RecyclingArrayList<FootstepTiming> recoveryTimings = new RecyclingArrayList<>(FootstepTiming::new);
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean isRecoveryImpossible = new YoBoolean("isRecoveryImpossible", registry);

   private final FramePoint2D leftToRightFootDistance = new FramePoint2D();
   private final DoubleProvider squareUpPreferredStanceWidth;
   private final DoubleProvider maxAllowedFinalStepXOffset;
   private final FootstepTiming squareUpStepTiming = new FootstepTiming();
   private final YoBoolean useRecoverySquareUpStep = new YoBoolean("useRecoverySquareUpStep", registry);

   public MultiStepPushRecoveryControlModule(SideDependentList<YoPlaneContactState> contactStates,
                                             BipedSupportPolygons bipedSupportPolygons,
                                             SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                             FrameConvexPolygon2DReadOnly defaultSupportPolygon,
                                             PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                             YoRegistry parentRegistry,
                                             YoGraphicsListRegistry graphicsListRegistry)
   {
      this.contactStates = contactStates;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.soleZUpFrames = soleZUpFrames;

      isICPOutside = new YoBoolean("isICPOutside", registry);
      isRobotBackToSafeState = new GlitchFilteredYoBoolean("isRobotBackToSafeState", registry, 100);

      swingSideForDoubleSupportRecovery = new YoEnum<>("swingSideForDoubleSupportRecovery", registry, RobotSide.class, true);
      swingSideForDoubleSupportRecovery.set(null);

      pushRecoveryTransferDuration = new DoubleParameter("pushRecoveryTransferDuration", registry, pushRecoveryControllerParameters.getRecoveryTransferDuration());
      pushRecoveryMinSwingDuration = new DoubleParameter("pushRecoveryMinSwingDuration", registry, pushRecoveryControllerParameters.getMinimumRecoverySwingDuration());
      pushRecoveryPreferredSwingDuration = new DoubleParameter("pushRecoveryPreferredSwingDuration", registry, pushRecoveryControllerParameters.getPreferredRecoverySwingDuration());
      pushRecoveryMaxSwingDuration = new DoubleParameter("pushRecoveryMaxSwingDuration", registry, pushRecoveryControllerParameters.getMaximumRecoverySwingDuration());



      DoubleParameter lengthLimit = new DoubleParameter("MaxReachabilityLength", registry, pushRecoveryControllerParameters.getMaxStepLength());
      DoubleParameter lengthBackLimit = new DoubleParameter("MaxReachabilityBackwardLength", registry, pushRecoveryControllerParameters.getMaxBackwardsStepLength());
      DoubleParameter innerLimit = new DoubleParameter("MinReachabilityWidth", registry, pushRecoveryControllerParameters.getMinStepWidth());
      DoubleParameter outerLimit = new DoubleParameter("MaxReachabilityWidth", registry, pushRecoveryControllerParameters.getMaxStepWidth());

      for (RobotSide robotSide : RobotSide.values)
      {
         pushRecoveryCalculators.put(robotSide, new MultiStepPushRecoveryCalculator(lengthLimit,
                                                                                    lengthBackLimit,
                                                                                    innerLimit,
                                                                                    outerLimit,
                                                                                    pushRecoveryControllerParameters,
                                                                                    soleZUpFrames,
                                                                                    defaultSupportPolygon));
         pushRecoveryCalculators.get(robotSide).setMaxStepsToGenerateForRecovery(pushRecoveryControllerParameters.getMaxStepsToGenerateForRecovery());
      }
      pushRecoveryCalculatorVisualizer = new MultiStepPushRecoveryCalculatorVisualizer("", 3, registry, graphicsListRegistry);

      IntegerParameter maxStepsToGenerateForRecovery = new IntegerParameter("maxStepsToGenerateForRecovery", registry, pushRecoveryControllerParameters.getMaxStepsToGenerateForRecovery());
      maxStepsToGenerateForRecovery.addListener((v) -> {
         for (RobotSide robotSide : RobotSide.values)
            pushRecoveryCalculators.get(robotSide).setMaxStepsToGenerateForRecovery(maxStepsToGenerateForRecovery.getValue());
      });

      squareUpPreferredStanceWidth = new DoubleParameter("squareUpPreferredStanceWidth", registry, pushRecoveryControllerParameters.getPreferredStepWidth());
      squareUpStepTiming.setTimings(pushRecoveryControllerParameters.getMinimumRecoverySwingDuration(), pushRecoveryControllerParameters.getRecoveryTransferDuration());
      maxAllowedFinalStepXOffset = new DoubleParameter("maxAllowedFinalStepXOffset", registry, pushRecoveryControllerParameters.getMaxAllowedFinalStepXOffset());

      useRecoverySquareUpStep.set(ENABLE_SQUARE_UP);
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      pushRecoveryCalculatorVisualizer.reset();
      recoveryFootsteps.clear();
      recoveryTimings.clear();
   }

   public RobotSide isRobotFallingFromDoubleSupport()
   {
      return swingSideForDoubleSupportRecovery.getEnumValue();
   }

   public int getNumberOfRecoverySteps()
   {
      return recoveryFootsteps.size();
   }

   public Footstep pollRecoveryStep()
   {
      Footstep footstep = recoveryFootsteps.get(0);
      recoveryFootsteps.remove(0);

      return footstep;
   }

   public FootstepTiming pollRecoveryStepTiming()
   {
      FootstepTiming timing = recoveryTimings.get(0);
      recoveryTimings.remove(0);

      return timing;
   }

   public Footstep getRecoveryStep(int stepIndex)
   {
      return recoveryFootsteps.get(stepIndex);
   }

   public FootstepTiming getRecoveryStepTiming(int stepIndex)
   {
      return recoveryTimings.get(stepIndex);
   }

   public boolean isRecoveryImpossible()
   {
      return isRecoveryImpossible.getValue();
   }

   public void updateForDoubleSupport(FramePoint2DReadOnly capturePoint2d, double omega0)
   {
      reset();

      // Initialize variables
      pushRecoveryCalculatorVisualizer.reset();
      swingSideForDoubleSupportRecovery.set(null);

      isICPOutside.set(!bipedSupportPolygons.getSupportPolygonInWorld().isPointInside(capturePoint2d));

      if (!isICPOutside.getBooleanValue())
      {
         isRobotBackToSafeState.update(true);
         isRecoveryImpossible.set(false);
         if(useRecoverySquareUpStep.getBooleanValue())
         {
            RobotSide nextSupportSide = computeSideOnCapturePoint(capturePoint2d);
            if(nextSupportSide == null)
               return;
            swingSideForDoubleSupportRecovery.set(nextSupportSide.getOppositeSide());

            MultiStepPushRecoveryCalculator pushRecoveryCalculator = pushRecoveryCalculators.get(nextSupportSide.getOppositeSide());
//            pushRecoveryCalculatorVisualizer.visualize(pushRecoveryCalculator);

            recoveryTimings.clear();
            recoveryFootsteps.clear();

            pushRecoveryCalculator.computeSquareUpStep(squareUpPreferredStanceWidth.getValue(), nextSupportSide, recoveryFootsteps.add());
            recoveryTimings.add().set(squareUpStepTiming);
         }
         return;
      }

      isRobotBackToSafeState.set(false);

      if (computeRecoveryStepLocations(capturePoint2d, omega0))
      {
         isRecoveryImpossible.set(false);
      }
      else
      {
         isRecoveryImpossible.set(true);
      }

      swingSideForDoubleSupportRecovery.set(computeBestRecoverySide());

      MultiStepPushRecoveryCalculator pushRecoveryCalculator = pushRecoveryCalculators.get(swingSideForDoubleSupportRecovery.getEnumValue());
      pushRecoveryCalculatorVisualizer.visualize(pushRecoveryCalculator);

      int numberOfSteps = pushRecoveryCalculator.getNumberOfRecoverySteps();

      for (int i = 0; i < numberOfSteps; i++)
      {
         recoveryFootsteps.add().set(pushRecoveryCalculator.getRecoveryStep(i));
         recoveryTimings.add().set(pushRecoveryCalculator.getRecoveryStepTiming(i));
      }
   }

   private RobotSide computeSideOnCapturePoint(FramePoint2DReadOnly capturePoint2d)
   {
      // first check if feet are already close to the preferred standing pose
      if(isRobotStanceCloseToPreferred())
         return null;

      if(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).isPointInside(capturePoint2d))
         return RobotSide.LEFT;
      else if(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT).isPointInside(capturePoint2d))
         return RobotSide.RIGHT;
      return null;
   }

   private boolean isRobotStanceCloseToPreferred()
   {
      FramePoint2DReadOnly leftFootCentroid = bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).getCentroid();
      FramePoint2DReadOnly rightFootCentroid = bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT).getCentroid();

      leftToRightFootDistance.setToZero();
      double xDistance = leftFootCentroid.getX() - rightFootCentroid.getX();
      double yDistance = leftFootCentroid.getY() - rightFootCentroid.getY();
      leftToRightFootDistance.set(xDistance, yDistance);
      leftToRightFootDistance.changeFrame(soleZUpFrames.get(RobotSide.RIGHT));
      return Math.abs(leftToRightFootDistance.getX()) < maxAllowedFinalStepXOffset.getValue();
   }

   public Footstep getFootstepForRecoveringFromDisturbance(double swingTimeRemaining)
   {
      return recoveryFootsteps.get(0);

      // TODO
//      checkAndUpdateFootstep(swingTimeRemaining, footstepToPack);
   }

   private boolean computeRecoveryStepLocations(FramePoint2DReadOnly currentICP, double omega0)
   {
      boolean isStateCapturable = false;
      for (RobotSide swingSide : RobotSide.values)
      {
         if (!contactStates.get(swingSide.getOppositeSide()).inContact())
            continue;

         supportPolygonInWorld.setIncludingFrame(bipedSupportPolygons.getFootPolygonInSoleFrame(swingSide.getOppositeSide()));
         supportPolygonInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

         MultiStepPushRecoveryCalculator pushRecoveryCalculator = pushRecoveryCalculators.get(swingSide);

         isStateCapturable |= pushRecoveryCalculator.computePreferredRecoverySteps(swingSide,
                                                                          pushRecoveryTransferDuration.getValue(),
                                                                          pushRecoveryPreferredSwingDuration.getValue(),
                                                                          currentICP,
                                                                          omega0,
                                                                          supportPolygonInWorld);
      }

      if (isStateCapturable)
         return isStateCapturable;

      for (RobotSide swingSide : RobotSide.values)
      {
         if (!contactStates.get(swingSide.getOppositeSide()).inContact())
            continue;

         supportPolygonInWorld.setIncludingFrame(bipedSupportPolygons.getFootPolygonInSoleFrame(swingSide.getOppositeSide()));
         supportPolygonInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

         MultiStepPushRecoveryCalculator pushRecoveryCalculator = pushRecoveryCalculators.get(swingSide);

         isStateCapturable |= pushRecoveryCalculator.computeRecoverySteps(swingSide,
                                                                                   pushRecoveryTransferDuration.getValue(),
                                                                                   pushRecoveryMinSwingDuration.getValue(),
                                                                                   pushRecoveryMaxSwingDuration.getValue(),
                                                                                   currentICP,
                                                                                   omega0,
                                                                                   supportPolygonInWorld);
      }
      return isStateCapturable;
   }

   private RobotSide computeBestRecoverySide()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!contactStates.get(robotSide).inContact())
            return robotSide;
      }

      boolean equalNumberOfSteps = pushRecoveryCalculators.get(RobotSide.LEFT).getNumberOfRecoverySteps() == pushRecoveryCalculators.get(RobotSide.RIGHT).getNumberOfRecoverySteps();

      if (!equalNumberOfSteps)
      {
         if (pushRecoveryCalculators.get(RobotSide.LEFT).getNumberOfRecoverySteps() < pushRecoveryCalculators.get(RobotSide.RIGHT).getNumberOfRecoverySteps())
            return RobotSide.LEFT;
         else
            return RobotSide.RIGHT;
      }

      double shortestStepLength = Double.MAX_VALUE;
      RobotSide shortestStepSide = null;

      for (RobotSide swingSide : RobotSide.values)
      {
         FramePoint2DReadOnly stepPosition = pushRecoveryCalculators.get(swingSide).getRecoveryStepLocation(0);
         double stepLength = bipedSupportPolygons.getFootPolygonInWorldFrame(swingSide).getCentroid().distance(stepPosition);

         if (stepLength < shortestStepLength)
         {
            shortestStepLength = stepLength;
            shortestStepSide = swingSide;
         }
      }

      return shortestStepSide;
   }

   /* TODO
   public boolean checkAndUpdateFootstep(double swingTimeRemaining, Footstep nextFootstep)
   {
      if (Double.isNaN(swingTimeRemaining))
         swingTimeRemaining = 1.0;

      RobotSide swingSide = nextFootstep.getRobotSide();
      RobotSide supportSide = swingSide.getOppositeSide();
      footPolygon.setIncludingFrame(bipedSupportPolygons.getFootPolygonInSoleZUpFrame(supportSide));

      double preferredSwingTimeForRecovering = computePreferredSwingTimeForRecovering(swingTimeRemaining, swingSide);
      captureRegionCalculator.calculateCaptureRegion(swingSide, preferredSwingTimeForRecovering, capturePoint2d, omega0, footPolygon);

      if (!isICPErrorTooLarge.getBooleanValue())
      {
         isRobotBackToSafeState.update(true);
         return false;
      }

      if (swingTimeRemaining < MINIMUM_TIME_TO_REPLAN)
      {
         // do not re-plan if we are almost at touch-down
         return false;
      }

      FramePoint2DReadOnly footCentroid = footPolygon.getCentroid();
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();
      isCaptureRegionEmpty.set(captureRegion.isEmpty());
      if (!recovering.getBooleanValue())
      {
         boolean hasFootstepBeenAdjusted = footstepAdjustor.adjustFootstep(nextFootstep, footCentroid, captureRegion);
         footstepWasProjectedInCaptureRegion.set(hasFootstepBeenAdjusted);
      }
      else
      {
         footstepWasProjectedInCaptureRegion.set(false);
      }

      if (footstepWasProjectedInCaptureRegion.getBooleanValue())
      {
         isRobotBackToSafeState.set(false);
         recovering.set(true);
      }

      return footstepWasProjectedInCaptureRegion.getBooleanValue();
   }
   */

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(pushRecoveryCalculatorVisualizer.getSCS2YoGraphics());
      return group;
   }
}
