package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataHolder;

import java.util.function.Supplier;

/**
 * Designed to avoid rebuilding the mesh unecessarily.
 * Call update every frame.
 */
public class RDXMutableLineModel extends RDXMutableMeshModel
{
   public static final double EPSILON = 1e-4;
   private final Point3D start = new Point3D();
   private final Point3D end = new Point3D();
   private double lineWidth = Double.NaN;

   private Supplier<MeshDataHolder> meshDataSupplier;
   private final MeshDataBuilder meshDataBuilder = new MeshDataBuilder();

   public void update(Tuple3DReadOnly start, Tuple3DReadOnly end, double lineWidth, Color color)
   {
      if (meshDataSupplier == null)
      {
         meshDataSupplier = () ->
         {
            meshDataBuilder.clear();
            meshDataBuilder.addLine(this.start, this.end, this.lineWidth);
            return meshDataBuilder.generateMeshDataHolder();
         };
      }

      boolean outOfDate = !this.start.geometricallyEquals(start, EPSILON);
      outOfDate |= !this.end.geometricallyEquals(end, EPSILON);
      outOfDate |= !EuclidCoreTools.epsilonEquals(this.lineWidth, lineWidth, EPSILON);
      outOfDate |= isColorOutOfDate(color);

      this.start.set(start);
      this.end.set(end);
      this.lineWidth = lineWidth;

      if (outOfDate)
      {
         updateMesh(meshDataSupplier.get());
      }
   }
}