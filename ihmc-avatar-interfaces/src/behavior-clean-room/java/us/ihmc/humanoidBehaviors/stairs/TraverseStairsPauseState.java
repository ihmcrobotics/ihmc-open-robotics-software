package us.ihmc.humanoidBehaviors.stairs;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicReference;

public class TraverseStairsPauseState implements State
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();

   public TraverseStairsPauseState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters)
   {
      this.helper = helper;
      this.parameters = parameters;

      helper.createROS2Callback(ROS2Tools.LIDAR_REA_REGIONS, planarRegions::set);
   }

   @Override
   public void onEntry()
   {
      LogTools.info("Entering " + getClass().getSimpleName());
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      boolean minimumPauseTimeElapsed = timeInState >= parameters.get(TraverseStairsBehaviorParameters.pauseTime);
      boolean receivedAnyRegions = planarRegions.get() != null;

      if (minimumPauseTimeElapsed && receivedAnyRegions)
      {
         LogTools.info(getClass().getSimpleName() + " is done");
         return true;
      }
      else if (!minimumPauseTimeElapsed)
      {
         LogTools.info(getClass().getSimpleName() + ": " + timeInState + "/" + parameters.get(TraverseStairsBehaviorParameters.pauseTime));
      }
      else if (!receivedAnyRegions)
      {
         LogTools.info(getClass().getSimpleName() + ": waiting for regions");
      }

      return false;
   }
}
