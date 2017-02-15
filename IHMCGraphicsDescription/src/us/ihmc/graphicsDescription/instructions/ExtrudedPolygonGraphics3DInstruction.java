package us.ihmc.graphicsDescription.instructions;

import javax.vecmath.Point2d;
import java.util.List;

public class ExtrudedPolygonGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final List<Point2d> polygonPoints;
   private final double extrusionHeight;

   public ExtrudedPolygonGraphics3DInstruction(List<Point2d> polygonPoints, double extrusionHeight)
   {
      this.polygonPoints = polygonPoints;
      this.extrusionHeight = extrusionHeight;
   }

   public List<Point2d> getPolygonPoints()
   {
      return polygonPoints;
   }

   public double getExtrusionHeight()
   {
      return extrusionHeight;
   }
}
