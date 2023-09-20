package us.ihmc.perception.sceneGraph;

import gnu.trove.map.hash.TIntObjectHashMap;

/**
 * A class to store the state of filtering candidate scene nodes.
 *
 * TODO: Make useful for different types of detections, not just ArUco
 *   marker detections.
 */
public class SceneGraphNodeCandidateFiltration
{
   private final TIntObjectHashMap<SceneGraphNodeCandidateFilter> markerIDFilters = new TIntObjectHashMap<>();

   public SceneGraphNodeCandidateFilter getOrCreateFilter(int markerID)
   {
      SceneGraphNodeCandidateFilter filter = markerIDFilters.get(markerID);

      if (filter == null)
      {
         filter = new SceneGraphNodeCandidateFilter();
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
