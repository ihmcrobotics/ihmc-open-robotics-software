package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class MultiStepPushRecoveryControlModule
{
   private static final boolean ENABLE_SQUARE_UP = false;

   private final GlitchFilteredYoBoolean isRobotBackToSafeState;
   private final YoBoolean isICPOutside;
   private final YoEnum<RobotSide> swingSideForDoubleSupportRecovery;

   private final SideDependentList<MultiStepPushRecoveryCalculator> pushRecoveryCalculators = new SideDependentList<>();
   private final MultiStepPushRecoveryCalculatorVisualizer pushRecoveryCalculatorVisualizer;

   private final SideDependentList<YoPlaneContactState> contactStates;
   private final BipedSupportPolygons bipedSupportPolygons;

   private final  DoubleProvider pushRecoveryTransferDuration;
   private final  DoubleProvider pushRecoverySwingDuration;

   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();
   private final RecyclingArrayList<Footstep> recoveryFootsteps = new RecyclingArrayList<>(Footstep::new);
   private final RecyclingArrayList<FootstepTiming> recoveryTimings = new RecyclingArrayList<>(FootstepTiming::new);
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean isRecoveryImpossible = new YoBoolean("isRecoveryImpossible", registry);

   private final DoubleProvider squareUpPreferredStanceWidth;
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

      isICPOutside = new YoBoolean("isICPOutside", registry);
      isRobotBackToSafeState = new GlitchFilteredYoBoolean("isRobotBackToSafeState", registry, 100);

      swingSideForDoubleSupportRecovery = new YoEnum<>("swingSideForDoubleSupportRecovery", registry, RobotSide.class, true);
      swingSideForDoubleSupportRecovery.set(null);

      pushRecoveryTransferDuration = new DoubleParameter("pushRecoveryTransferDuration", registry, pushRecoveryControllerParameters.getRecoveryTransferDuration());
      pushRecoverySwingDuration = new DoubleParameter("pushRecoverySwingDuration", registry, pushRecoveryControllerParameters.getRecoverySwingDuration());



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

      squareUpPreferredStanceWidth = new DoubleParameter("squareUpPreferredStanceWidth", registry, pushRecoveryControllerParameters.getPreferredFinalStepWidth());
      squareUpStepTiming.setTimings(pushRecoveryControllerParameters.getRecoverySwingDuration(), pushRecoveryControllerParameters.getRecoveryTransferDuration());

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
            recoveryFootsteps.add().set(pushRecoveryCalculator.computeSquareUpStep(squareUpPreferredStanceWidth.getValue(), nextSupportSide));
            recoveryTimings.add().set(squareUpStepTiming);
         }
         return;
      }

      isRobotBackToSafeState.set(false);

      if (computeRecoveryStepLocations(capturePoint2d, omega0))
      {
         swingSideForDoubleSupportRecovery.set(computeBestRecoverySide());
         isRecoveryImpossible.set(false);

         MultiStepPushRecoveryCalculator pushRecoveryCalculator = pushRecoveryCalculators.get(swingSideForDoubleSupportRecovery.getEnumValue());
         pushRecoveryCalculatorVisualizer.visualize(pushRecoveryCalculator);

         int numberOfSteps = pushRecoveryCalculator.getNumberOfRecoverySteps();

         for (int i = 0; i < numberOfSteps; i++)
         {
            recoveryFootsteps.add().set(pushRecoveryCalculator.getRecoveryStep(i));
            recoveryTimings.add().set(pushRecoveryCalculator.getRecoveryStepTiming(i));
         }
      }
      else
      {
         isRecoveryImpossible.set(true);
      }
   }

   private RobotSide computeSideOnCapturePoint(FramePoint2DReadOnly capturePoint2d)
   {
      // first check if feet are already close to the preferred standing pose
//      if(isRobotStanceCloseToPreferred())
//         return null;

      if(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).isPointInside(capturePoint2d))
         return RobotSide.LEFT;
      else if(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT).isPointInside(capturePoint2d))
         return RobotSide.RIGHT;
      return null;
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

         bipedSupportPolygons.getFootPolygonInSoleFrame(swingSide.getOppositeSide());
         supportPolygonInWorld.setIncludingFrame(bipedSupportPolygons.getFootPolygonInSoleFrame(swingSide.getOppositeSide()));
         supportPolygonInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

         MultiStepPushRecoveryCalculator pushRecoveryCalculator = pushRecoveryCalculators.get(swingSide);

         isStateCapturable |= pushRecoveryCalculator.computeRecoverySteps(swingSide,
                                                                          pushRecoverySwingDuration.getValue(),
                                                                          pushRecoveryTransferDuration.getValue(),
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

}
