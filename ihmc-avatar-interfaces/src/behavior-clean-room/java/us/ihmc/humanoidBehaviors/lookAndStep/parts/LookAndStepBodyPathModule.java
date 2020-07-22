package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.Timer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.lookAndStep.*;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.humanoidBehaviors.tools.walking.WalkingFootstepTracker;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.function.Supplier;

public class LookAndStepBodyPathModule extends LookAndStepBodyPathTask
{
   private final Supplier<RemoteSyncedRobotModel> robotStateSupplier;
   private final WalkingFootstepTracker walkingFootstepTracker;

   private final TypedInput<PlanarRegionsList> mapRegionsInput = new TypedInput<>();
   private final TypedInput<Pose3D> goalInput = new TypedInput<>();
   private final Timer mapRegionsExpirationTimer = new Timer();
   private final Timer planningFailedTimer = new Timer();

   // hook up inputs and notifications separately.
   // always run again if with latest data
   public LookAndStepBodyPathModule(StatusLogger statusLogger,
                                    UIPublisher uiPublisher,
                                    VisibilityGraphsParametersReadOnly visibilityGraphParameters,
                                    LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                                    Supplier<Boolean> operatorReviewEnabled,
                                    RemoteSyncedRobotModel syncedRobot,
                                    BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference,
                                    Supplier<Boolean> robotConnectedSupplier,
                                    WalkingFootstepTracker walkingFootstepTracker)
   {
      super(statusLogger,
            uiPublisher,
            visibilityGraphParameters,
            lookAndStepBehaviorParameters,
            operatorReviewEnabled,
            behaviorStateReference,
            robotConnectedSupplier);

      this.robotStateSupplier = () -> {
         syncedRobot.update();
         return syncedRobot;
      };
      this.walkingFootstepTracker = walkingFootstepTracker;

      // don't run two body path plans at the same time
      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      mapRegionsInput.addCallback(data -> executor.execute(this::evaluateAndRun));
      goalInput.addCallback(data -> executor.execute(this::evaluateAndRun));

      setResetPlanningFailedTimer(planningFailedTimer::reset);
   }

   public void acceptMapRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      mapRegionsInput.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      mapRegionsExpirationTimer.reset();
   }

   public void acceptGoal(Pose3D goal)
   {
      goalInput.set(goal);
      LogTools.info("Body path goal received: {}", goal);
   }

   private void evaluateAndRun()
   {
      update(mapRegionsInput.get(),
             goalInput.get(),
             robotStateSupplier.get(),
             mapRegionsExpirationTimer.createSnapshot(lookAndStepBehaviorParameters.getPlanarRegionsExpiration()),
             planningFailedTimer.createSnapshot(lookAndStepBehaviorParameters.getWaitTimeAfterPlanFailed()),
             behaviorStateReference.get(),
             walkingFootstepTracker.getNumberOfIncompleteFootsteps());

      run();
   }
}
