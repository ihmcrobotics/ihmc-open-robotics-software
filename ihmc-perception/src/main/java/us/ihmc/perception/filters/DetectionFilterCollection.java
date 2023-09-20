package us.ihmc.perception.filters;

import gnu.trove.map.hash.TIntObjectHashMap;

/**
 * A class to store the state of filtering candidate scene nodes.
 *
 * TODO: Make useful for different types of detections, not just ArUco
 *   marker detections.
 */
public class DetectionFilterCollection
{
   private final TIntObjectHashMap<DetectionFilter> markerIDFilters = new TIntObjectHashMap<>();

   public DetectionFilter getOrCreateFilter(int markerID)
   {
      DetectionFilter filter = markerIDFilters.get(markerID);

      if (filter == null)
      {
         filter = new DetectionFilter();
         markerIDFilters.put(markerID, filter);
      }

      return filter;
   }

   public void update()
   {
      for (int markerID : markerIDFilters.keys())
      {
         markerIDFilters.get(markerID).update();
      }
   }

   public void removeFilter(int markerID)
   {
      markerIDFilters.remove(markerID);
   }
}
