package us.ihmc.quadrupedPlanning.velocityPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedPlanning.pathPlanning.SplinePathPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedBodyPathPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SplinePathPlanner waypointPlanner = new SplinePathPlanner(registry);
   protected final WaypointDefinedBodyPathPlanner bodyPathPlanner = new WaypointDefinedBodyPathPlanner();
   private final QuadrupedConstantAccelerationBodyPathPlanner quadBodyPathPlanner;

   public QuadrupedBodyPathPlanner(YoVariableRegistry parentRegistry)
   {
      quadBodyPathPlanner = new QuadrupedConstantAccelerationBodyPathPlanner(registry);

      parentRegistry.addChild(registry);
   }

   public void setInitialBodyPose(FramePose3DReadOnly bodyPose)
   {
      waypointPlanner.setInitialBodyPose(bodyPose);
   }

   public void setGoal(QuadrupedFootstepPlannerGoal goal)
   {
      waypointPlanner.setGoal(goal);
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      waypointPlanner.setPlanarRegionsList(planarRegionsList);
   }

   public void compute()
   {
      waypointPlanner.planWaypoints();

      bodyPathPlanner.setWaypoints(waypointPlanner.getWaypoints());
      bodyPathPlanner.compute();

      quadBodyPathPlanner.setBodyPathWaypoints(bodyPathPlanner.getPlan());
      quadBodyPathPlanner.computePlan();
   }

   public QuadrupedBodyPathPlan getPlan()
   {
      return quadBodyPathPlanner.getPlan();
   }
}
