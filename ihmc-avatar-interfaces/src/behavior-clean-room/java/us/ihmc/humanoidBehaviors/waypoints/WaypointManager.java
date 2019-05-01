package us.ihmc.humanoidBehaviors.waypoints;

import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;

public class WaypointManager // should handle comms of waypointsequence, unique id management (creating waypoints)
{
   private final Messager messager;
   private final Topic<WaypointSequence> sendTopic;

   private volatile WaypointSequence activeSequence = new WaypointSequence();

   private long highestSeenId = -1;

   private final Topic<Integer> waypointIndexUIUpdateTopic;
   private final boolean delayUpdate; // for module to
   private volatile WaypointSequence delayedUpdateSequence;

   public static WaypointManager createForModule(Messager messager,
                                                 Topic<WaypointSequence> receiveTopic,
                                                 Topic<WaypointSequence> sendTopic,
                                                 Topic<Integer> goToWaypointTopic,
                                                 Topic<Integer> waypointIndexUIUpdateTopic,
                                                 Notification goNotification)
   {
      WaypointManager waypointManager = new WaypointManager(messager,
                                                            receiveTopic,
                                                            sendTopic,
                                                            waypointIndexUIUpdateTopic,
                                                            null,
                                                            true);
      messager.registerTopicListener(goToWaypointTopic, goToWaypointIndex -> // easy to put here, so do
      {
         LogTools.info("Recieved GoToWaypoint {}", goToWaypointIndex);
         waypointManager.updateToMostRecentData();
         waypointManager.setNextFromIndex(goToWaypointIndex);
         goNotification.set();
      });
      return waypointManager;
   }

   public static WaypointManager createForUI(Messager messager,
                                             Topic<WaypointSequence> receiveTopic,
                                             Topic<WaypointSequence> sendTopic,
                                             Runnable receivedListener) // accept listenerForUI
   {
      WaypointManager waypointManager = new WaypointManager(messager,
                                                            receiveTopic,
                                                            sendTopic,
                                                            null,
                                                            receivedListener,
                                                            false);
      return waypointManager;
   }

   public WaypointManager(Messager messager,
                          Topic<WaypointSequence> receiveTopic,
                          Topic<WaypointSequence> sendTopic,
                          Topic<Integer> waypointIndexUIUpdateTopic,
                          Runnable receivedListener,
                          boolean delayUpdate)
   {
      this.messager = messager;
      this.sendTopic = sendTopic;
      this.waypointIndexUIUpdateTopic = waypointIndexUIUpdateTopic;
      this.delayUpdate = delayUpdate;

      messager.registerTopicListener(receiveTopic, newSequence ->
      {
         LogTools.info("Received {} updated waypoints.", newSequence.size());
         if (delayUpdate)
         {
            delayedUpdateSequence = newSequence;
         }
         else
         {
            activeSequence = newSequence;
         }

         updateHighestSeenId();

         if (receivedListener != null)
         {
            receivedListener.run();
         }
      });
   }

   public void publish()
   {
      LogTools.info("Publishing active waypoint sequence: size: {} id: {} {}", activeSequence.size(),
                    activeSequence.peekNext().getUniqueId(),
                    activeSequence);
      messager.submitMessage(sendTopic, activeSequence);
   }

   public void clearWaypoints()
   {
      activeSequence.clear();
   }

   public void increment()
   {
      activeSequence.increment();
      publishNextIndexToUI();
   }

   /** For UI to set by user seeing continuous number line */
   public void setNextFromIndex(int index)
   {
      activeSequence.setNextFromIndex(index);
      publishNextIndexToUI();
   }

   private void publishNextIndexToUI()
   {
      if (waypointIndexUIUpdateTopic != null)
      {
         messager.submitMessage(waypointIndexUIUpdateTopic, activeSequence.peekNextIndex());
      }
   }

   public boolean hasWaypoints()
   {
      return !activeSequence.isEmpty();
   }

   public int size()
   {
      return activeSequence.size();
   }

   public int indexOfId(long id)
   {
      return activeSequence.indexOfId(id);
   }

   public long idFromIndex(int index)
   {
      return activeSequence.get(index).getUniqueId();
   }

   public Waypoint appendNewWaypoint()
   {
      Waypoint newWaypoint = newWaypoint();
      activeSequence.add(newWaypoint);
      return newWaypoint;
   }

   public Waypoint insertNewWaypoint(int insertionIndex)
   {
      Waypoint insertedWaypoint = newWaypoint();
      activeSequence.insert(insertionIndex, insertedWaypoint);
      return insertedWaypoint;
   }

   public void remove(long id)
   {
      activeSequence.remove(id);
   }

   private Waypoint newWaypoint()
   {
      return new Waypoint(++highestSeenId);
   }

   public void updateToMostRecentData()
   {
      if (delayUpdate)
      {
         activeSequence = delayedUpdateSequence;
      }
   }

   public boolean incrementingWillLoop()
   {
      return activeSequence.incrementingWillLoop();
   }

   public Pose3DReadOnly peekNextPose()
   {
      return activeSequence.peekNext().getPose();
   }

   public int peekNextIndex()
   {
      return activeSequence.peekNextIndex();
   }

   public Pose3DReadOnly peekAfterNextPose()
   {
      return activeSequence.peekAfterNext().getPose();
   }

   public Pose3DBasics getPoseFromId(long id)
   {
      return activeSequence.get(indexOfId(id)).getPose();
   }

   public long lastId()
   {
      return activeSequence.last().getUniqueId();
   }

   private void updateHighestSeenId()
   {
      long highestId = activeSequence.highestId();
      if (highestId > highestSeenId)
      {
         highestSeenId = highestId;
      }
   }
}
