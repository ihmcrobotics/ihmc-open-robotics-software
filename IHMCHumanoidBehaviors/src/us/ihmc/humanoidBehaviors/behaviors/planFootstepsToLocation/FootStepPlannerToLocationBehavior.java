package us.ihmc.humanoidBehaviors.behaviors.planFootstepsToLocation;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootStepPlannerToLocationBehavior extends AbstractBehavior
{
   private final FootstepPlannerGoal goal;
   private final FramePose goalPose = new FramePose();
   private final FootstepPlanner planner;
   private final FramePose initialPose = new FramePose();

   private final BooleanYoVariable isFootstepPlanAvailable = new BooleanYoVariable("isFootstepPlanAvailable", registry);

   private RobotSide initialSide;
   private FootstepPlanningResult footstepPlanningResult;
   private FootstepPlan footstepPlan;
   FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();


   public FootStepPlannerToLocationBehavior(CommunicationBridgeInterface communicationBridge)
   {
      super(communicationBridge);

      planner = new TurnWalkTurnPlanner();
      goal = new FootstepPlannerGoal();

   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doControl()
   {
      footstepPlanningResult = planner.plan();

      if(footstepPlanningResult.validForExecution())
      {
         footstepPlan = planner.getPlan();
         
      }


   }

   @Override
   public void initialize()
   {
      goal.setGoalPoseBetweenFeet(goalPose);
      planner.setGoal(goal);
      planner.setInitialStanceFoot(initialPose, initialSide);

   }

   public void setInitialPose(FramePose initialPose)
   {
      this.initialPose.setIncludingFrame(initialPose);
   }

   public void setInitialSide(RobotSide initialSide)
   {
      this.initialSide = initialSide;
   }

   public void setGoalPose(FramePose goalPose)
   {
      this.goalPose.setIncludingFrame(goalPose);
   }
}
