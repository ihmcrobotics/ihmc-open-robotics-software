package us.ihmc.behaviors.stairs;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class TraverseStairsPauseState extends TraverseStairsState
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();
   private final AtomicBoolean firstTick = new AtomicBoolean(true);
   private static final double pauseDurationOnFirstTick = 3.0;
   private final StatusLogger statusLogger;

   public TraverseStairsPauseState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters)
   {
      this.helper = helper;
      this.parameters = parameters;
      this.statusLogger = helper.getOrCreateStatusLogger();

      helper.subscribeViaCallback(PerceptionAPI.LIDAR_REA_REGIONS, planarRegions::set);
   }

   public void reset()
   {
      firstTick.set(true);
   }

   @Override
   public void onEntry()
   {
      statusLogger.info("Entering " + getClass().getSimpleName() + ". Pausing for " + getPauseDuration() + "sec");

      boolean wasPreviouslyMoving = getPreviousStateName() == TraverseStairsBehavior.TraverseStairsStateName.EXECUTE_STEPS || getPreviousStateName() == TraverseStairsBehavior.TraverseStairsStateName.SQUARE_UP;
      if (firstTick.get() || wasPreviouslyMoving)
      {
         REAStateRequestMessage clearMessage = new REAStateRequestMessage();
         clearMessage.setRequestClear(true);
         helper.publish(PerceptionAPI.REA_STATE_REQUEST, clearMessage);
      }
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      boolean minimumPauseTimeElapsed = timeInState >= getPauseDuration();
      boolean receivedAnyRegions = planarRegions.get() != null;

      if (minimumPauseTimeElapsed && receivedAnyRegions)
      {
         statusLogger.info(getClass().getSimpleName() + " is done");
         return true;
      }
      else if (!minimumPauseTimeElapsed)
      {
         double totalPauseDuration = parameters.get(TraverseStairsBehaviorParameters.pauseTime);
//         helper.publish(TimeLeftInPause, totalPauseDuration - timeInState);
      }
      else if (!receivedAnyRegions)
      {
         statusLogger.info(getClass().getSimpleName() + ": waiting for regions");
      }

      return false;
   }

   @Override
   public void onExit(double timeInState)
   {
      firstTick.set(false);
   }

   private double getPauseDuration()
   {
      return firstTick.get() ? pauseDurationOnFirstTick : parameters.get(TraverseStairsBehaviorParameters.pauseTime);
   }
}
