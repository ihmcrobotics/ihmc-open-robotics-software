package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.HashMap;

public class PlanarRegionConstraintDataHolder
{
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();

   private final HashMap<PlanarRegion, PlanarRegionConstraintData> planarRegionConstraintData = new HashMap<>();

   public void clear()
   {
      planarRegionConstraintData.clear();
   }

   public ConvexPolygon2DReadOnly getScaledRegionPolygon(PlanarRegion planarRegion, Point2DReadOnly pointInLocal, PlanarRegionConstraintDataParameters parameters)
   {
      PlanarRegionConstraintData constraintData = getConstraintData(planarRegion, parameters);
      return constraintData.getScaledRegionPolygon(pointInLocal);
   }

   public TIntArrayList getIndicesToExclude(PlanarRegion planarRegion, Point2DReadOnly pointInLocal, PlanarRegionConstraintDataParameters parameters)
   {
      PlanarRegionConstraintData constraintData = getConstraintData(planarRegion, parameters);
      return constraintData.getPolygonIndicesToIgnore(pointInLocal);
   }

   private PlanarRegionConstraintData getConstraintData(PlanarRegion planarRegion, PlanarRegionConstraintDataParameters parameters)
   {
      PlanarRegionConstraintData constraintData;
      if (planarRegionConstraintData.containsKey(planarRegion))
      {
         constraintData = planarRegionConstraintData.get(planarRegion);
      }
      else
      {
         constraintData = new PlanarRegionConstraintData(polygonScaler, planarRegion, parameters.projectInsideUsingConvexHull, parameters.projectionInsideDelta);
         planarRegionConstraintData.put(planarRegion, constraintData);
      }

      return constraintData;
   }

}
