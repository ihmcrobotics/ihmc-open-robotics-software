package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class XGaitCost implements FootstepCost
{
   private final FootstepPlannerParameters plannerParameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public XGaitCost(FootstepPlannerParameters plannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.plannerParameters = plannerParameters;
      this.xGaitSettings = xGaitSettings;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant();
      RobotQuadrant previousQuadrant = startNode.getMovingQuadrant();
      if (movingQuadrant.getNextReversedRegularGaitSwingQuadrant() != previousQuadrant)
      {
         throw new RuntimeException("For some reason the feet movement is out of order.");
      }

      double phaseShift = xGaitSettings.getEndPhaseShift();

      Point2DReadOnly startXGaitCenter = startNode.getOrComputeXGaitCenterPoint();

      double durationBetweenSteps = computeTimeDeltaBetweenSteps(previousQuadrant);
      double desiredSpeed = plannerParameters.getDesiredWalkingSpeed(phaseShift);
      double desiredDistance = durationBetweenSteps * desiredSpeed;


      return 0;
   }

   private double computeTimeDeltaBetweenSteps(RobotQuadrant previousQuadrant)
   {
      double phaseShift = xGaitSettings.getEndPhaseShift();
      double phaseTimeShift = phaseShift / 180.0 * xGaitSettings.getStepDuration();

      if (previousQuadrant.isQuadrantInFront())
      {
         return phaseTimeShift;
      }
      else
      {
         return phaseTimeShift + xGaitSettings.getEndDoubleSupportDuration();
      }
   }
}
