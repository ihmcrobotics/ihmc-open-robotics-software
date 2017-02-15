package us.ihmc.graphicsDescription.instructions;

import javax.vecmath.Point3d;
import java.util.List;

public class PolygonGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final List<Point3d> polygonPoints;

   public PolygonGraphics3DInstruction(List<Point3d> polygonPoints)
   {
      this.polygonPoints = polygonPoints;
   }

   public List<Point3d> getPolygonPoints()
   {
      return polygonPoints;
   }
}
