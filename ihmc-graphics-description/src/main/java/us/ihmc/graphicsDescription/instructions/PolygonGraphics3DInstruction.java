package us.ihmc.graphicsDescription.instructions;

import us.ihmc.euclid.tuple3D.Point3D;
import java.util.List;

public class PolygonGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final List<Point3D> polygonPoints;

   public PolygonGraphics3DInstruction(List<Point3D> polygonPoints)
   {
      this.polygonPoints = polygonPoints;
   }

   public List<Point3D> getPolygonPoints()
   {
      return polygonPoints;
   }
}
