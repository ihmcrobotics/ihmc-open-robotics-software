package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.MeshDataHolder;

import java.util.function.Supplier;

/**
 * Designed to avoid rebuilding the mesh unecessarily.
 * Call update every frame.
 */
public class RDXMutableArrowModel extends RDXMutableMeshModel
{
   public static final double EPSILON = 1e-4;
   private double length = Double.NaN;

   private Supplier<MeshDataHolder> meshDataSupplier;

   public void update(double lengthUpdate, Color color)
   {
      if (meshDataSupplier == null)
      {
         meshDataSupplier = () -> MeshDataGeneratorMissing.Arrow(length);
      }

      boolean outOfDate = !EuclidCoreTools.epsilonEquals(length, lengthUpdate, EPSILON);
      outOfDate |= isColorOutOfDate(color);

      this.length = lengthUpdate;

      if (outOfDate)
      {
         updateMesh(meshDataSupplier.get());
      }
   }
}