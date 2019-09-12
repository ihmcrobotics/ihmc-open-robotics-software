package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.HashMap;

public class PlanarRegionPawConstraintDataHolder
{
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();

   private final HashMap<PlanarRegion, PlanarRegionPawConstraintData> planarRegionConstraintData = new HashMap<>();

   public void clear()
   {
      planarRegionConstraintData.clear();
   }

   public ConvexPolygon2DReadOnly getScaledRegionPolygon(PlanarRegion planarRegion, Point2DReadOnly pointInLocal, PlanarRegionPawConstraintDataParameters parameters)
   {
      PlanarRegionPawConstraintData constraintData = getConstraintData(planarRegion, parameters);
      return constraintData.getScaledRegionPolygon(pointInLocal);
   }

   public TIntArrayList getIndicesToExclude(PlanarRegion planarRegion, ConvexPolygon2DReadOnly containingRegion,
                                            PlanarRegionPawConstraintDataParameters parameters)
   {
      PlanarRegionPawConstraintData constraintData = getConstraintData(planarRegion, parameters);
      return constraintData.getPolygonIndicesToIgnore(containingRegion);
   }

   private PlanarRegionPawConstraintData getConstraintData(PlanarRegion planarRegion, PlanarRegionPawConstraintDataParameters parameters)
   {
      PlanarRegionPawConstraintData constraintData;
      if (planarRegionConstraintData.containsKey(planarRegion))
      {
         constraintData = planarRegionConstraintData.get(planarRegion);
      }
      else
      {
         constraintData = new PlanarRegionPawConstraintData(polygonScaler, planarRegion, parameters.projectInsideUsingConvexHull, parameters.projectionInsideDelta);
         planarRegionConstraintData.put(planarRegion, constraintData);
      }

      return constraintData;
   }

}
