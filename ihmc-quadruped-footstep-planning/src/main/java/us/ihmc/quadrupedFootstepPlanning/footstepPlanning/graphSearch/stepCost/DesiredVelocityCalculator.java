package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;

public class DesiredVelocityCalculator
{
   private final FootstepPlannerParameters plannerParameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   private final FramePose3D goalPose = new FramePose3D();
   private final double turningMultiplier = 0.5;

   public DesiredVelocityCalculator(FootstepPlannerParameters plannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.plannerParameters = plannerParameters;
      this.xGaitSettings = xGaitSettings;
   }

   public void setGoal(FramePose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   private final Vector2D planarVelocity = new Vector2D();
   public Vector3DReadOnly getDesiredVelocityForNode(FootstepNode node)
   {
      double desiredSpeed = plannerParameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double turningSpeed = turningMultiplier * desiredSpeed;

      planarVelocity.set(goalPose.getPosition());
      planarVelocity.sub(node.getOrComputeXGaitCenterPoint());
      planarVelocity.normalize();
      planarVelocity.scale(desiredSpeed);

      Vector3D velocity = new Vector3D();
      velocity.set(planarVelocity);
      velocity.setZ(turningSpeed);
   }
}
