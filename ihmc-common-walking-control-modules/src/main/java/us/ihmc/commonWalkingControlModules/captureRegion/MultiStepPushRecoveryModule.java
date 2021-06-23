package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.IntFunction;

public class MultiStepPushRecoveryModule
{
   private static final boolean ENABLE_SQUARE_UP = false;

   private final YoBoolean isICPOutside;
   private final YoEnum<RobotSide> swingSideForDoubleSupportRecovery;

   private final SideDependentList<? extends FrameConvexPolygon2DReadOnly> footPolygonsInWorld;
   private final SideDependentList<MultiStepRecoveryStepCalculator> pushRecoveryCalculators = new SideDependentList<>();
   private final SquareUpStepCalculator squareUpStepCalculator;
   private final MultiStepPushRecoveryCalculatorVisualizer pushRecoveryCalculatorVisualizer;

   private final Function<RobotSide, Boolean> isInContact;

   private final  DoubleProvider pushRecoveryTransferDuration;
   private final  DoubleProvider pushRecoveryPreferredSwingDuration;
   private final  DoubleProvider pushRecoveryMinSwingDuration;
   private final  DoubleProvider pushRecoveryMaxSwingDuration;

   private final FrameConvexPolygon2DReadOnly supportPolygonInWorld;
   private final RecyclingArrayList<Footstep> recoveryFootsteps = new RecyclingArrayList<>(Footstep::new);
   private final RecyclingArrayList<FootstepTiming> recoveryTimings = new RecyclingArrayList<>(FootstepTiming::new);
   private final List<PlanarRegion> constraintRegions = new ArrayList<>();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean isRecoveryImpossible = new YoBoolean("isRecoveryImpossible", registry);

   private final YoBoolean useRecoverySquareUpStep = new YoBoolean("useRecoverySquareUpStep", registry);

   public MultiStepPushRecoveryModule(Function<RobotSide, Boolean> isInContact,
                                      FrameConvexPolygon2DReadOnly supportPolygonInWorld,
                                      SideDependentList<? extends FrameConvexPolygon2DReadOnly> footPolygonsInWorld,
                                      SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                      ConvexPolygon2DReadOnly defaultSupportPolygon,
                                      PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                      YoRegistry parentRegistry,
                                      YoGraphicsListRegistry graphicsListRegistry)
   {
      this.isInContact = isInContact;
      this.supportPolygonInWorld = supportPolygonInWorld;
      this.footPolygonsInWorld = footPolygonsInWorld;

      isICPOutside = new YoBoolean("isICPOutside", registry);

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
         pushRecoveryCalculators.put(robotSide, new MultiStepRecoveryStepCalculator(lengthLimit,
                                                                                    lengthBackLimit,
                                                                                    innerLimit,
                                                                                    outerLimit,
                                                                                    pushRecoveryControllerParameters,
                                                                                    soleZUpFrames,
                                                                                    defaultSupportPolygon));
         pushRecoveryCalculators.get(robotSide).setMaxStepsToGenerateForRecovery(pushRecoveryControllerParameters.getMaxStepsToGenerateForRecovery());
      }
      if (graphicsListRegistry != null)
         pushRecoveryCalculatorVisualizer = new MultiStepPushRecoveryCalculatorVisualizer("", 3, registry, graphicsListRegistry);
      else
         pushRecoveryCalculatorVisualizer = null;

      squareUpStepCalculator = new SquareUpStepCalculator(footPolygonsInWorld, soleZUpFrames, pushRecoveryControllerParameters, registry);

      IntegerParameter maxStepsToGenerateForRecovery = new IntegerParameter("maxStepsToGenerateForRecovery", registry, pushRecoveryControllerParameters.getMaxStepsToGenerateForRecovery());
      maxStepsToGenerateForRecovery.addListener((v) -> {
         for (RobotSide robotSide : RobotSide.values)
            pushRecoveryCalculators.get(robotSide).setMaxStepsToGenerateForRecovery(maxStepsToGenerateForRecovery.getValue());
      });

      useRecoverySquareUpStep.set(ENABLE_SQUARE_UP);
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      recoveryFootsteps.clear();
      recoveryTimings.clear();
      constraintRegions.clear();
      isRecoveryImpossible.set(false);
      if (pushRecoveryCalculatorVisualizer != null)
         pushRecoveryCalculatorVisualizer.reset();
   }

   public void setConstraintRegionProvider(IntFunction<PlanarRegionsList> constraintRegionProvider)
   {
      for (RobotSide robotSide : RobotSide.values)
         pushRecoveryCalculators.get(robotSide).setConstraintRegionProvider(constraintRegionProvider);
   }

   public void setPlanarRegionDecider(CapturabilityBasedPlanarRegionDecider<PlanarRegion> planarRegionDecider)
   {
      for (RobotSide robotSide : RobotSide.values)
         pushRecoveryCalculators.get(robotSide).setPlanarRegionDecider(planarRegionDecider);
   }

   public boolean isRobotFallingFromDoubleSupport()
   {
      return getSwingSideForRecovery() != null;
   }

   public RobotSide getSwingSideForRecovery()
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

   public boolean hasConstraintRegions()
   {
      return !constraintRegions.isEmpty();
   }

   public PlanarRegion getConstraintRegion(int stepIndex)
   {
      return constraintRegions.get(stepIndex);
   }

   public boolean isRecoveryImpossible()
   {
      return isRecoveryImpossible.getValue();
   }

   public void updateForDoubleSupport(FramePoint2DReadOnly capturePoint2d, double omega0)
   {
      reset();

      // Initialize variables
      if (pushRecoveryCalculatorVisualizer != null)
         pushRecoveryCalculatorVisualizer.reset();
      swingSideForDoubleSupportRecovery.set(null);

      isICPOutside.set(!supportPolygonInWorld.isPointInside(capturePoint2d));

      if (!isICPOutside.getBooleanValue())
      {
         isRecoveryImpossible.set(false);

         if (useRecoverySquareUpStep.getBooleanValue())
         {
            RobotSide nextSupportSide = squareUpStepCalculator.computeSideOnCapturePoint(capturePoint2d);
            if (nextSupportSide == null)
               return;
            swingSideForDoubleSupportRecovery.set(nextSupportSide.getOppositeSide());

            recoveryTimings.clear();
            recoveryFootsteps.clear();

            squareUpStepCalculator.computeSquareUpStep(nextSupportSide, recoveryFootsteps.add());
            recoveryTimings.add().set(squareUpStepCalculator.getSquareUpStepTiming());
         }
         return;
      }

      isRecoveryImpossible.set(!computeRecoveryStepLocations(capturePoint2d, omega0));
      if (isRecoveryImpossible.getBooleanValue())
         return;

      swingSideForDoubleSupportRecovery.set(computeBestRecoverySide());

      MultiStepRecoveryStepCalculator pushRecoveryCalculator = pushRecoveryCalculators.get(swingSideForDoubleSupportRecovery.getEnumValue());
      if (pushRecoveryCalculatorVisualizer != null)
         pushRecoveryCalculatorVisualizer.visualize(pushRecoveryCalculator);

      int numberOfSteps = pushRecoveryCalculator.getNumberOfRecoverySteps();

      for (int i = 0; i < numberOfSteps; i++)
      {
         recoveryFootsteps.add().set(pushRecoveryCalculator.getRecoveryStep(i));
         recoveryTimings.add().set(pushRecoveryCalculator.getRecoveryStepTiming(i));
         if (pushRecoveryCalculator.hasConstraintRegions())
            constraintRegions.add(pushRecoveryCalculator.getConstraintRegion(i));
      }
   }

   private boolean computeRecoveryStepLocations(FramePoint2DReadOnly currentICP, double omega0)
   {
      boolean isStateCapturable = false;
      for (RobotSide swingSide : RobotSide.values)
      {
         if (!isInContact.apply(swingSide.getOppositeSide()))
            continue;

         MultiStepRecoveryStepCalculator pushRecoveryCalculator = pushRecoveryCalculators.get(swingSide);

         isStateCapturable |= pushRecoveryCalculator.computePreferredRecoverySteps(swingSide,
                                                                                   pushRecoveryTransferDuration.getValue(),
                                                                                   pushRecoveryPreferredSwingDuration.getValue(),
                                                                                   currentICP,
                                                                                   omega0,
                                                                                   footPolygonsInWorld.get(swingSide.getOppositeSide()));
      }

      if (isStateCapturable)
         return isStateCapturable;

      for (RobotSide swingSide : RobotSide.values)
      {
         if (!isInContact.apply(swingSide.getOppositeSide()))
            continue;

         MultiStepRecoveryStepCalculator pushRecoveryCalculator = pushRecoveryCalculators.get(swingSide);

         isStateCapturable |= pushRecoveryCalculator.computeRecoverySteps(swingSide,
                                                                          pushRecoveryTransferDuration.getValue(),
                                                                          pushRecoveryMinSwingDuration.getValue(),
                                                                          pushRecoveryMaxSwingDuration.getValue(),
                                                                          currentICP,
                                                                          omega0,
                                                                          footPolygonsInWorld.get(swingSide.getOppositeSide()));
      }
      return isStateCapturable;
   }

   private RobotSide computeBestRecoverySide()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isInContact.apply(robotSide))
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
         double stepLength = footPolygonsInWorld.get(swingSide).getCentroid().distance(stepPosition);

         if (stepLength < shortestStepLength)
         {
            shortestStepLength = stepLength;
            shortestStepSide = swingSide;
         }
      }

      return shortestStepSide;
   }
}
