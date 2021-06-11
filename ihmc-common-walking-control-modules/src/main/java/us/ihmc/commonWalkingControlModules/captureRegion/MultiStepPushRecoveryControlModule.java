package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
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
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public class MultiStepPushRecoveryControlModule
{
   private static final double swingDuration = 0.6;
   private static final double transferDuration = 0.1;

   private final GlitchFilteredYoBoolean isRobotBackToSafeState;
   private final YoBoolean isICPOutside;
   private final YoEnum<RobotSide> swingSideForDoubleSupportRecovery;

   private final MultiStepPushRecoveryCalculator pushRecoveryCalculator;

   private final BipedSupportPolygons bipedSupportPolygons;

   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();
   private final SideDependentList<RecyclingArrayList<Footstep>> recoveryFootsteps = new SideDependentList<>(new RecyclingArrayList<>(Footstep::new),
                                                                                                             new RecyclingArrayList<>(Footstep::new));
   private final SideDependentList<RecyclingArrayList<FootstepTiming>> recoveryTimings = new SideDependentList<>(new RecyclingArrayList<>(FootstepTiming::new),
                                                                                                                 new RecyclingArrayList<>(FootstepTiming::new));

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   public MultiStepPushRecoveryControlModule(BipedSupportPolygons bipedSupportPolygons,
                                             SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                             FrameConvexPolygon2DReadOnly defaultSupportPolygon,
                                             YoRegistry parentRegistry,
                                             YoGraphicsListRegistry graphicsListRegistry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;

      isICPOutside = new YoBoolean("isICPOutside", registry);
      isRobotBackToSafeState = new GlitchFilteredYoBoolean("isRobotBackToSafeState", registry, 100);

      swingSideForDoubleSupportRecovery = new YoEnum<>("swingSideForDoubleSupportRecovery", registry, RobotSide.class, true);
      swingSideForDoubleSupportRecovery.set(null);

      double footWidth = 0.1;
      double kinematicsStepRange = 1.0;
      pushRecoveryCalculator = new MultiStepPushRecoveryCalculator(kinematicsStepRange,
                                                                   footWidth,
                                                                   soleZUpFrames,
                                                                   defaultSupportPolygon,
                                                                   "",
                                                                   registry,
                                                                   graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public RobotSide isRobotFallingFromDoubleSupport()
   {
      return swingSideForDoubleSupportRecovery.getEnumValue();
   }

   public int getNumberOfRecoverySteps(RobotSide firstSwingSide)
   {
      return recoveryFootsteps.get(firstSwingSide).size();
   }

   public Footstep getRecoveryStep(RobotSide firstSwingSide, int stepIndex)
   {
      return recoveryFootsteps.get(firstSwingSide).get(stepIndex);
   }

   public FootstepTiming getRecoveryStepTiming(RobotSide firstSwingSide, int stepIndex)
   {
      return recoveryTimings.get(firstSwingSide).get(stepIndex);
   }

   public void updateForDoubleSupport(FramePoint2DReadOnly capturePoint2d, double omega0)
   {
      // Initialize variables
      swingSideForDoubleSupportRecovery.set(null);

      isICPOutside.set(!bipedSupportPolygons.getSupportPolygonInWorld().isPointInside(capturePoint2d));

      if (!isICPOutside.getBooleanValue())
      {
         isRobotBackToSafeState.update(true);
         return;
      }

      isRobotBackToSafeState.set(false);

      computeRecoveryStepLocations(capturePoint2d, omega0);

      swingSideForDoubleSupportRecovery.set(computeBestRecoverySide());
   }

   private RobotSide computeRecoveryStepLocations(FramePoint2DReadOnly currentICP, double omega0)
   {
      int minNumberOfSteps = Integer.MAX_VALUE;
      RobotSide minStepSwingSide = null;

      for (RobotSide swingSide : RobotSide.values)
      {
         recoveryFootsteps.get(swingSide).clear();
         recoveryTimings.get(swingSide).clear();

         bipedSupportPolygons.getFootPolygonInSoleFrame(swingSide.getOppositeSide());
         supportPolygonInWorld.setIncludingFrame(bipedSupportPolygons.getFootPolygonInSoleFrame(swingSide.getOppositeSide()));
         supportPolygonInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

         int numberOfSteps = pushRecoveryCalculator.calculateRecoveryStepLocations(swingSide, swingDuration, transferDuration, currentICP, omega0,
                                                                                   supportPolygonInWorld);
         for (int i = 0; i < numberOfSteps; i++)
         {
            recoveryFootsteps.get(swingSide).add().set(pushRecoveryCalculator.getRecoveryStep(i));
            recoveryTimings.get(swingSide).add().set(pushRecoveryCalculator.getRecoveryStepTiming(i));

         }

         if (numberOfSteps < minNumberOfSteps)
         {
            minNumberOfSteps = numberOfSteps;
            minStepSwingSide = swingSide;
         }
      }

      return minStepSwingSide;
   }

   private RobotSide computeBestRecoverySide()
   {
      boolean equalNumberOfSteps = recoveryFootsteps.get(RobotSide.LEFT).size() == recoveryFootsteps.get(RobotSide.RIGHT).size();

      if (!equalNumberOfSteps)
      {
         if (recoveryFootsteps.get(RobotSide.LEFT).size() < recoveryFootsteps.get(RobotSide.RIGHT).size())
            return RobotSide.LEFT;
         else
            return RobotSide.RIGHT;
      }

      double shortestStepLength = Double.MAX_VALUE;
      RobotSide shortestStepSide = null;

      for (RobotSide swingSide : RobotSide.values)
      {
         FramePoint3DReadOnly stepPosition = recoveryFootsteps.get(swingSide).get(0).getFootstepPose().getPosition();
         double stepLength = bipedSupportPolygons.getFootPolygonInWorldFrame(swingSide).getCentroid().distanceXY(stepPosition);

         if (stepLength < shortestStepLength)
         {
            shortestStepLength = stepLength;
            shortestStepSide = swingSide;
         }
      }

      return shortestStepSide;
   }

}
