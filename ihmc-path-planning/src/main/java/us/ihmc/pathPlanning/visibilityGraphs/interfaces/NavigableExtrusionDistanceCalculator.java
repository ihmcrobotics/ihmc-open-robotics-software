package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.robotics.geometry.PlanarRegion;

public interface NavigableExtrusionDistanceCalculator
{
   /**
    * Computes the extrusion distance to use when extrusion the hull of this navigable region.
    * 
    * @param navigableRegionToBeExtruded the navigable region being extruded. Do not modify.
    * @return the extrusion distance to use.
    */
   double computeExtrusionDistance(PlanarRegion navigableRegionToBeExtruded);
}
