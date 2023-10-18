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

   public DetectionFilter createFilter(int markerID, int history)
   {
      DetectionFilter filter = new DetectionFilter(history);
      markerIDFilters.put(markerID, filter);
      return filter;
   }

   public DetectionFilter getFilter(int markerID)
   {
      return markerIDFilters.get(markerID);
   }

   public void removeFilter(int markerID)
   {
      markerIDFilters.remove(markerID);
   }
}
