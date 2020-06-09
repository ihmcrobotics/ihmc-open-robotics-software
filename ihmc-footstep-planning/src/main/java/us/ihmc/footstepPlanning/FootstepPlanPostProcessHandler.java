package us.ihmc.footstepPlanning;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.icp.AreaBasedSplitFractionCalculator;
import us.ihmc.footstepPlanning.icp.PositionBasedSplitFractionCalculator;
import us.ihmc.footstepPlanning.icp.SplitFractionCalculatorParametersBasics;
import us.ihmc.footstepPlanning.swing.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.function.Consumer;

public class FootstepPlanPostProcessHandler
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SwingPlannerParametersBasics swingPlannerParameters;
   private final SplitFractionCalculatorParametersBasics splitFractionParameters;

   private final AdaptiveSwingTrajectoryCalculator adaptiveSwingTrajectoryCalculator;
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;

   private final AreaBasedSplitFractionCalculator areaBasedSplitFractionCalculator;
   private final PositionBasedSplitFractionCalculator positionBasedSplitFractionCalculator;

   private Consumer<FootstepPlannerOutput> statusCallback = result -> {};

   public FootstepPlanPostProcessHandler(SwingPlannerParametersBasics swingPlannerParameters,
                                         SplitFractionCalculatorParametersBasics splitFractionParameters,
                                         WalkingControllerParameters walkingControllerParameters,
                                         SideDependentList<ConvexPolygon2D> footPolygons)

   {
      this.swingPlannerParameters = swingPlannerParameters;
      this.splitFractionParameters = splitFractionParameters;

      if (walkingControllerParameters == null)
      {
         this.adaptiveSwingTrajectoryCalculator = null;
         this.swingOverPlanarRegionsTrajectoryExpander = null;
      }
      else
      {
         this.adaptiveSwingTrajectoryCalculator = new AdaptiveSwingTrajectoryCalculator(swingPlannerParameters, walkingControllerParameters);
         this.swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters,
                                                                                                      registry,
                                                                                                      new YoGraphicsListRegistry());
      }

      this.areaBasedSplitFractionCalculator = new AreaBasedSplitFractionCalculator(splitFractionParameters, footPolygons);
      this.positionBasedSplitFractionCalculator = new PositionBasedSplitFractionCalculator(splitFractionParameters);
   }

   public void handleRequest(FootstepPlannerRequest request, FootstepPlannerOutput output)
   {
      boolean flatGroundMode = request.getAssumeFlatGround() || request.getPlanarRegionsList() == null || request.getPlanarRegionsList().isEmpty();
      if (flatGroundMode)
      {
         statusCallback.accept(output);
         return;
      }

      if (request.getSwingPlannerType() == SwingPlannerType.PROPORTION && adaptiveSwingTrajectoryCalculator != null)
      {
         adaptiveSwingTrajectoryCalculator.setPlanarRegionsList(request.getPlanarRegionsList());
         RobotSide initialStanceSide = request.getRequestedInitialStanceSide();
         Pose3D initialStancePose = request.getStartFootPoses().get(initialStanceSide);
         adaptiveSwingTrajectoryCalculator.setSwingParameters(initialStancePose, output.getFootstepPlan());
      }
      else if (request.getSwingPlannerType() == SwingPlannerType.POSITION && swingOverPlanarRegionsTrajectoryExpander != null)
      {
         computeSwingWaypoints(request, output.getFootstepPlan());
      }

      if (request.performPositionBasedSplitFractionCalculation())
      {
         positionBasedSplitFractionCalculator.computeSplitFractions(request, output.getFootstepPlan());
      }
      if (request.performAreaBasedSplitFractionCalculation())
      {
         areaBasedSplitFractionCalculator.computeSplitFractions(request, output.getFootstepPlan());
      }

      statusCallback.accept(output);
   }


   // TODO make this a method of the swing trajectory solver after moving it to this package
   public void computeSwingWaypoints(FootstepPlannerRequest request, FootstepPlan footstepPlan)
   {
      swingOverPlanarRegionsTrajectoryExpander.setDoInitialFastApproximation(swingPlannerParameters.getDoInitialFastApproximation());
      swingOverPlanarRegionsTrajectoryExpander.setFastApproximationLessClearance(swingPlannerParameters.getFastApproximationLessClearance());
      swingOverPlanarRegionsTrajectoryExpander.setNumberOfCheckpoints(swingPlannerParameters.getNumberOfChecksPerSwing());
      swingOverPlanarRegionsTrajectoryExpander.setMaximumNumberOfTries(swingPlannerParameters.getMaximumNumberOfAdjustmentAttempts());
      swingOverPlanarRegionsTrajectoryExpander.setMinimumSwingFootClearance(swingPlannerParameters.getMinimumSwingFootClearance());
      swingOverPlanarRegionsTrajectoryExpander.setMinimumAdjustmentIncrementDistance(swingPlannerParameters.getMinimumAdjustmentIncrementDistance());
      swingOverPlanarRegionsTrajectoryExpander.setMaximumAdjustmentIncrementDistance(swingPlannerParameters.getMaximumAdjustmentIncrementDistance());
      swingOverPlanarRegionsTrajectoryExpander.setAdjustmentIncrementDistanceGain(swingPlannerParameters.getAdjustmentIncrementDistanceGain());
      swingOverPlanarRegionsTrajectoryExpander.setMaximumAdjustmentDistance(swingPlannerParameters.getMaximumWaypointAdjustmentDistance());
      swingOverPlanarRegionsTrajectoryExpander.setMinimumHeightAboveFloorForCollision(swingPlannerParameters.getMinimumHeightAboveFloorForCollision());

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         FramePose3D swingEndPose = new FramePose3D(footstep.getFootstepPose());
         FramePose3D swingStartPose = new FramePose3D();
         FramePose3D stanceFootPose = new FramePose3D();

         RobotSide swingSide = footstep.getRobotSide();
         RobotSide stanceSide = swingSide.getOppositeSide();

         if (i == 0)
         {
            swingStartPose.set(request.getStartFootPoses().get(swingSide));
            stanceFootPose.set(request.getStartFootPoses().get(stanceSide));
         }
         else if (i == 1)
         {
            swingStartPose.set(request.getStartFootPoses().get(swingSide));
            stanceFootPose.set(footstepPlan.getFootstep(i - 1).getFootstepPose());
         }
         else
         {
            swingStartPose.set(footstepPlan.getFootstep(i - 2).getFootstepPose());
            stanceFootPose.set(footstepPlan.getFootstep(i - 1).getFootstepPose());
         }

         double maxSpeedDimensionless = swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose,
                                                                                                                   swingStartPose,
                                                                                                                   swingEndPose,
                                                                                                                   request.getPlanarRegionsList());
         if (swingOverPlanarRegionsTrajectoryExpander.wereWaypointsAdjusted())
         {
            footstep.setTrajectoryType(TrajectoryType.CUSTOM);
            Point3D waypointOne = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0));
            Point3D waypointTwo = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1));
            footstep.setCustomWaypointPositions(waypointOne, waypointTwo);

            System.out.println("step " + i + " was adjusted.");
            System.out.println("\t waypoint 0: " + waypointOne);
            System.out.println("\t waypoint 1: " + waypointTwo);

            // TODO scale swing time
         }
      }
   }

   public void setStatusCallback(Consumer<FootstepPlannerOutput> statusCallback)
   {
      this.statusCallback = statusCallback;
   }

   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return swingPlannerParameters;
   }

   public SplitFractionCalculatorParametersBasics getSplitFractionParameters()
   {
      return splitFractionParameters;
   }

   public AdaptiveSwingTrajectoryCalculator getAdaptiveSwingTrajectoryCalculator()
   {
      return adaptiveSwingTrajectoryCalculator;
   }

   public SwingOverPlanarRegionsTrajectoryExpander getSwingOverPlanarRegionsTrajectoryExpander()
   {
      return swingOverPlanarRegionsTrajectoryExpander;
   }

   public AreaBasedSplitFractionCalculator getAreaBasedSplitFractionCalculator()
   {
      return areaBasedSplitFractionCalculator;
   }

   public PositionBasedSplitFractionCalculator getPositionBasedSplitFractionCalculator()
   {
      return positionBasedSplitFractionCalculator;
   }
}
