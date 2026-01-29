package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataHolder;

import java.util.List;
import java.util.function.Supplier;

/**
 * Designed to avoid rebuilding the mesh unecessarily.
 * Call update every frame.
 */
public class RDXMutableMultiLineModel extends RDXMutableMeshModel
{
   public static final double EPSILON = 1e-4;
   public static final int DEFAULT_MAX_POINTS = 500; // Reduced from 1000 for better performance
   private static final int MESH_UPDATE_THROTTLE = 5; // Only rebuild mesh every N frames
   private static final double MIN_POINT_DISTANCE = 0.05; // Minimum distance between points (5cm) to avoid excessive density
   private final RecyclingArrayList<Point3D> points = new RecyclingArrayList<>(Point3D::new);
   private double lineWidth = Double.NaN;
   private int maxPoints = DEFAULT_MAX_POINTS;
   private int framesSinceLastMeshUpdate = MESH_UPDATE_THROTTLE; // Start ready to update

   private Supplier<MeshDataHolder> meshDataSupplier;
   private final MeshDataBuilder meshDataBuilder = new MeshDataBuilder();

   public void update(List<? extends Tuple3DReadOnly> pointsUpdateReadOnly, double lineWidthUpdate, Color color)
   {
      if (meshDataSupplier == null)
      {
         meshDataSupplier = () ->
         {
            meshDataBuilder.clear();
            boolean close = false;
            meshDataBuilder.addMultiLine(points, lineWidth, close);
            return meshDataBuilder.generateMeshDataHolder();
         };
      }

      boolean outOfDate = false;

      outOfDate |= modelInstance == null != pointsUpdateReadOnly.isEmpty();
      outOfDate |= points.size() != pointsUpdateReadOnly.size();
      outOfDate |= !EuclidCoreTools.epsilonEquals(lineWidth, lineWidthUpdate, EPSILON);
      outOfDate |= isColorOutOfDate(color);

      // Optimization: Only check last few points instead of all points
      // Since we're adding points incrementally, only the newest points change
      if (!outOfDate && points.size() == pointsUpdateReadOnly.size() && points.size() > 0)
      {
         // Only check the last 5 points
         int checkCount = Math.min(5, points.size());
         int startCheck = points.size() - checkCount;

         for (int i = startCheck; i < points.size(); i++)
         {
            if (!points.get(i).geometricallyEquals(pointsUpdateReadOnly.get(i), EPSILON))
            {
               outOfDate = true;
               break;
            }
         }
      }

      if (outOfDate)
      {
         // Increment throttle counter
         framesSinceLastMeshUpdate++;

         // Only rebuild expensive mesh every N frames
         if (framesSinceLastMeshUpdate >= MESH_UPDATE_THROTTLE)
         {
            framesSinceLastMeshUpdate = 0;

            points.clear();

            // Determine which points to keep based on max limit
            int startIndex = Math.max(0, pointsUpdateReadOnly.size() - maxPoints);

            // Decimate points that are too close together by averaging/skipping
            if (pointsUpdateReadOnly.size() > startIndex)
            {
               // Always add the first point
               points.add().set(pointsUpdateReadOnly.get(startIndex));

               for (int i = startIndex + 1; i < pointsUpdateReadOnly.size(); i++)
               {
                  Tuple3DReadOnly currentPoint = pointsUpdateReadOnly.get(i);
                  Point3D lastAddedPoint = points.get(points.size() - 1);

                  double distanceToLast = lastAddedPoint.distanceXY((Point3DReadOnly) currentPoint) + Math.abs(lastAddedPoint.getZ() - currentPoint.getZ());

                  // Only add point if it's far enough from the last one
                  if (distanceToLast >= MIN_POINT_DISTANCE)
                  {
                     points.add().set(currentPoint);
                  }
                  // If it's the last point, always add it to ensure trail reaches current position
                  else if (i == pointsUpdateReadOnly.size() - 1)
                  {
                     points.add().set(currentPoint);
                  }
               }
            }

            lineWidth = lineWidthUpdate;

            updateMesh(meshDataSupplier.get());
         }
      }
      else
      {
         // Reset throttle counter when data is up to date
         framesSinceLastMeshUpdate = 0;
      }
   }

   public void setMaxPoints(int maxPoints)
   {
      this.maxPoints = maxPoints;
   }

   public int getMaxPoints()
   {
      return maxPoints;
   }
}