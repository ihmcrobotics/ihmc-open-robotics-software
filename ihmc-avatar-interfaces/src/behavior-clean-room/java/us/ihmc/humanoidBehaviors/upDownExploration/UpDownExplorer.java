package us.ihmc.humanoidBehaviors.upDownExploration;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.humanoidBehaviors.upDownExploration.UpDownSequence.UpDown;
import us.ihmc.humanoidBehaviors.waypoints.Waypoint;
import us.ihmc.humanoidBehaviors.waypoints.WaypointManager;
import us.ihmc.messager.Messager;
import us.ihmc.tools.thread.TypedNotification;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI.UpDownCenter;
import static us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI.UpDownExplorationEnabled;

/**
 * Keep track of state and manage the specific flow of exploration for the May 2019 demo.
 *
 * Flow is after reset, go up or down. if went down, turn 180. if went up, turn random.
 *
 */
public class UpDownExplorer
{
   private final UpDownFlatAreaFinder upDownFlatAreaFinder;
   private TypedNotification<Optional<FramePose3D>> planNotification = new TypedNotification<>();
   private final AtomicReference<Double> exploreTurnAmount;
   private final AtomicReference<Point3D> upDownCenter;

   private double accumulatedTurnAmount = 0.0;

   private RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private ROS2Input<PlanarRegionsListMessage> planarRegionsList;

   enum UpDownState
   {
      NO_STATE,
      JUST_WENT_UP,
      JUST_WENT_DOWN,
      JUST_TURNED,

   }

   private UpDownState state = UpDownState.JUST_WENT_DOWN;

   public UpDownExplorer(Messager messager, AtomicReference<Boolean> upDownExplorationEnabled,
                         AtomicReference<Double> exploreTurnAmount,
                         RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames,
                         ROS2Input<PlanarRegionsListMessage> planarRegionsList)
   {
      this.exploreTurnAmount = exploreTurnAmount;
      this.remoteSyncedHumanoidFrames = remoteSyncedHumanoidFrames;
      this.planarRegionsList = planarRegionsList;

      upDownFlatAreaFinder = new UpDownFlatAreaFinder(messager);

      messager.registerTopicListener(UpDownExplorationEnabled, enabled -> { if (enabled) state = UpDownState.NO_STATE; });
      upDownCenter = messager.createInput(UpDownCenter, new Point3D(0.0, 0.0, 0.0));
   }

   public void onNavigateEntry()
   {
      // TODO this should plan only if

      if (shouldLookForUpDown())
      {
         planNotification = upDownFlatAreaFinder.upOrDownOnAThread(remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames().getMidFeetZUpFrame(),
                                                                   PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsList.getLatest()));
      }
   }

   private boolean shouldLookForUpDown()
   {
      return state == UpDownState.NO_STATE || state == UpDownState.JUST_WENT_DOWN || state == UpDownState.JUST_TURNED;
   }

   public void poll()
   {
      if (shouldLookForUpDown())
      {
         planNotification.poll();
      }
   }

   public boolean shouldTransitionToPlan()
   {
      return !shouldLookForUpDown() || planNotification.hasNext();
   }

   public void onPlanEntry(FramePose3DReadOnly midFeetZUpPose, WaypointManager waypointManager)
   {
      waypointManager.clearWaypoints();
      Waypoint newWaypoint = waypointManager.appendNewWaypoint();

      if (shouldLookForUpDown()) // going to what the updown found
      {
         if (planNotification.peek().isPresent()) // success
         {
            newWaypoint.getPose().set(planNotification.peek().get());

            UpDown foundUpOrDown = upDownFlatAreaFinder.getLastPlanUpDown();
            if (foundUpOrDown == UpDown.UP)
            {
               state = UpDownState.JUST_WENT_UP;
            }
            else
            {
               state = UpDownState.JUST_WENT_DOWN;
            }

         }
         else
         {
            // problem: assume doesn't happen
            newWaypoint.getPose().set(midFeetZUpPose);
         }
      }
      else // just went up
      {
         newWaypoint.getPose().set(midFeetZUpPose);
         double amountToTurn = Math.toRadians(exploreTurnAmount.get());

         if (accumulatedTurnAmount > 0.0)
         {
            amountToTurn = -Math.abs(amountToTurn);
         }
         else
         {
            amountToTurn = Math.abs(amountToTurn);
         }

         accumulatedTurnAmount += amountToTurn;
         newWaypoint.getPose().appendYawRotation(amountToTurn);

         state = UpDownState.JUST_TURNED;
      }

      waypointManager.publish();
      waypointManager.setNextFromIndex(0);
   }

   public boolean shouldTransitionFromPerceive()
   {
      return !shouldLookForUpDown();
   }

   public void abortPlanning()
   {
      upDownFlatAreaFinder.abort();
   }

   public TypedNotification<Optional<FramePose3D>> getPlanNotification()
   {
      return planNotification;
   }
}
