package us.ihmc.humanoidBehaviors.waypoints;

import us.ihmc.log.LogTools;

import java.util.ArrayList;

/**
 * This is a custom and kinda weird data structure. It's a wrapped list with a "next" pointer
 * that refers not to the index, but the unique ID of the object stored at that index. This
 * is so next can adapt to insertion and removal of objects around the list.
 */
public class WaypointSequence
{
   private ArrayList<Waypoint> list = new ArrayList<>();
   private long nextWaypointUniqueId = -1;

   public int peekNextIndex()
   {
      return indexOfId(nextWaypointUniqueId);
   }

   public int indexOfId(long id) // TODO restructure to rid this search?
   {
      int currentIndex = 0;
      for (int i = 0; i < list.size(); i++)
      {
         if (list.get(i).getUniqueId() == id)
         {
            currentIndex = i;
            break;  // assume only 1 matches
         }
      }
      return currentIndex;
   }

   public void clear()
   {
      list.clear();
      nextWaypointUniqueId = -1;
   }

   public boolean isEmpty()
   {
      return list.isEmpty();
   }

   public int size()
   {
      return list.size();
   }

   public Waypoint last()
   {
      return list.get(list.size() - 1);
   }

   public void add(Waypoint newWaypoint)
   {
      list.add(newWaypoint);
   }

   public void remove(long id) // TODO handle missing waypoint for id
   {
      list.remove(indexOfId(id));
   }

   public Waypoint get(int index)
   {
      return list.get(index);
   }

   public void insert(int insertionIndex, Waypoint waypointToInsert)
   {
      list.add(insertionIndex, waypointToInsert);
   }

   public Waypoint peekNext()
   {
      return list.get(peekNextIndex());
   }

   public Waypoint peekAfterNext()
   {
      return list.get(incrementedIndex());
   }

   public void setNextFromIndex(int index) // assume no out of bounds
   {
      nextWaypointUniqueId = list.get(index).getUniqueId();
      LogTools.warn("Set next id to {}, index: {}", nextWaypointUniqueId, indexOfId(nextWaypointUniqueId));
   }

   public long highestId()
   {
      long highestId = -1;
      for (Waypoint waypoint : list)
      {
         if (waypoint.getUniqueId() > highestId)
         {
            highestId = waypoint.getUniqueId();
         }
      }
      return highestId;
   }

   public void increment()
   {
      int incrementedIndex = incrementedIndex();
      nextWaypointUniqueId = list.get(incrementedIndex).getUniqueId();
      LogTools.warn("Increment next id to {}, index: {}", nextWaypointUniqueId, indexOfId(nextWaypointUniqueId));
   }

   public boolean incrementingWillLoop()
   {
      return peekNextIndex() + 1 == list.size();
   }

   private int incrementedIndex()
   {
      int incrementedIndex = peekNextIndex() + 1;
      LogTools.warn("Candidate index: {}", incrementedIndex);
      if (incrementedIndex == list.size())
      {
         incrementedIndex = 0;
      }
      LogTools.warn("Incremented index: {}, list.size: {}", incrementedIndex, list.size());
      return incrementedIndex;
   }
}
