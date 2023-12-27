package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
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
   private final RecyclingArrayList<Point3D> points = new RecyclingArrayList<>(Point3D::new);
   private double lineWidth = Double.NaN;

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

      outOfDate |= points.size() != pointsUpdateReadOnly.size();
      outOfDate |= !EuclidCoreTools.epsilonEquals(lineWidth, lineWidthUpdate, EPSILON);
      outOfDate |= isColorOutOfDate(color);

      if (!outOfDate) // Only iterate if list sizes are the same
      {
         for (int i = 0; i < pointsUpdateReadOnly.size(); i++)
         {
            outOfDate |= !points.get(i).geometricallyEquals(pointsUpdateReadOnly.get(i), EPSILON);
         }
      }

      if (outOfDate)
      {
         points.clear();

         for (Tuple3DReadOnly point : pointsUpdateReadOnly)
         {
            points.add().set(point);
         }

         lineWidth = lineWidthUpdate;
      }

      if (outOfDate)
      {
         updateMesh(meshDataSupplier.get());
      }
   }
}