package us.ihmc.footstepPlanning;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.footstepPlanning.swing.CollisionFreeSwingCalculator;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingPlanningModule
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final SwingPlannerParametersBasics swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   private final AdaptiveSwingTrajectoryCalculator adaptiveSwingTrajectoryCalculator;
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
   private final CollisionFreeSwingCalculator collisionFreeSwingCalculator;

   private double nominalSwingTrajectoryLength;

   public SwingPlanningModule(FootstepPlannerParametersReadOnly footstepPlannerParameters,
                              SwingPlannerParametersBasics swingPlannerParameters,
                              WalkingControllerParameters walkingControllerParameters,
                              SideDependentList<ConvexPolygon2D> footPolygons)

   {
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.swingPlannerParameters = swingPlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;

      if (walkingControllerParameters == null)
      {
         this.adaptiveSwingTrajectoryCalculator = null;
         this.swingOverPlanarRegionsTrajectoryExpander = null;
         this.collisionFreeSwingCalculator = null;
      }
      else
      {
         this.adaptiveSwingTrajectoryCalculator = new AdaptiveSwingTrajectoryCalculator(swingPlannerParameters, footstepPlannerParameters, walkingControllerParameters);
         this.swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters,
                                                                                                      registry,
                                                                                                      new YoGraphicsListRegistry());
         this.collisionFreeSwingCalculator = new CollisionFreeSwingCalculator(footstepPlannerParameters,
                                                                              swingPlannerParameters,
                                                                              walkingControllerParameters,
                                                                              footPolygons);
      }
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void computeSwingWaypoints(PlanarRegionsList planarRegionsList,
                                     FootstepPlan footstepPlan,
                                     SideDependentList<? extends Pose3DReadOnly> startFootPoses,
                                     SwingPlannerType swingPlannerType)
   {
      if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return;
      }

      if (swingPlannerType == SwingPlannerType.PROPORTION && adaptiveSwingTrajectoryCalculator != null)
      {
         adaptiveSwingTrajectoryCalculator.setPlanarRegionsList(planarRegionsList);
         adaptiveSwingTrajectoryCalculator.setSwingParameters(startFootPoses, footstepPlan);
      }
      else if (swingPlannerType == SwingPlannerType.TWO_WAYPOINT_POSITION && swingOverPlanarRegionsTrajectoryExpander != null)
      {
         computeSwingWaypoints(planarRegionsList, footstepPlan, startFootPoses);
      }
      else if (swingPlannerType == SwingPlannerType.MULTI_WAYPOINT_POSITION && collisionFreeSwingCalculator != null)
      {
         collisionFreeSwingCalculator.setPlanarRegionsList(planarRegionsList);
         collisionFreeSwingCalculator.computeSwingTrajectories(startFootPoses, footstepPlan);
      }
   }

   // TODO make this a method of the swing trajectory solver after moving it to this package
   private void computeSwingWaypoints(PlanarRegionsList planarRegionsList,
                                     FootstepPlan footstepPlan,
                                     SideDependentList<? extends Pose3DReadOnly> startFootPoses)
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

      computeNominalSwingTrajectoryLength();

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
            swingStartPose.set(startFootPoses.get(swingSide));
            stanceFootPose.set(startFootPoses.get(stanceSide));
         }
         else if (i == 1)
         {
            swingStartPose.set(startFootPoses.get(swingSide));
            stanceFootPose.set(footstepPlan.getFootstep(i - 1).getFootstepPose());
         }
         else
         {
            swingStartPose.set(footstepPlan.getFootstep(i - 2).getFootstepPose());
            stanceFootPose.set(footstepPlan.getFootstep(i - 1).getFootstepPose());
         }

         swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose,
                                                                                    swingStartPose,
                                                                                    swingEndPose,
                                                                                    planarRegionsList);
         if (swingOverPlanarRegionsTrajectoryExpander.wereWaypointsAdjusted())
         {
            footstep.setTrajectoryType(TrajectoryType.CUSTOM);
            Point3D waypointOne = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0));
            Point3D waypointTwo = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1));
            footstep.setCustomWaypointPositions(waypointOne, waypointTwo);

            double swingScale = Math.max(1.0, swingOverPlanarRegionsTrajectoryExpander.getExpandedTrajectoryLength() / nominalSwingTrajectoryLength);
            double swingTime = swingPlannerParameters.getAdditionalSwingTimeIfExpanded() + swingScale * swingPlannerParameters.getMinimumSwingTime();
            footstep.setSwingDuration(swingTime);
         }
         else
         {
            double swingScale = Math.max(1.0, swingOverPlanarRegionsTrajectoryExpander.getInitialTrajectoryLength() / nominalSwingTrajectoryLength);
            double swingTime = swingScale * swingPlannerParameters.getMinimumSwingTime();
            footstep.setSwingDuration(swingTime);
         }
      }
   }

   private void computeNominalSwingTrajectoryLength()
   {
      double nominalSwingHeight = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();
      double nominalSwingProportion = TwoWaypointSwingGenerator.getDefaultWaypointProportions()[0];
      double nominalSwingLength = 2.0 * footstepPlannerParameters.getIdealFootstepLength();

      // trapezoidal approximation of swing trajectory
      nominalSwingTrajectoryLength = 2.0 * EuclidGeometryTools.pythagorasGetHypotenuse(nominalSwingProportion * nominalSwingLength, nominalSwingHeight)
                                     + (1.0 - 2.0 * nominalSwingProportion) * nominalSwingLength;
   }

   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return swingPlannerParameters;
   }

   public AdaptiveSwingTrajectoryCalculator getAdaptiveSwingTrajectoryCalculator()
   {
      return adaptiveSwingTrajectoryCalculator;
   }

   public SwingOverPlanarRegionsTrajectoryExpander getSwingOverPlanarRegionsTrajectoryExpander()
   {
      return swingOverPlanarRegionsTrajectoryExpander;
   }
}
