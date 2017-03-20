package us.ihmc.graphicsDescription.instructions;

import java.util.List;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class ExtrudedPolygonGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final List<? extends Point2DReadOnly> polygonPoints;
   private final double extrusionHeight;

   public ExtrudedPolygonGraphics3DInstruction(List<? extends Point2DReadOnly> polygonPoints, double extrusionHeight)
   {
      this.polygonPoints = polygonPoints;
      this.extrusionHeight = extrusionHeight;
   }

   public List<? extends Point2DReadOnly> getPolygonPoints()
   {
      return polygonPoints;
   }

   public double getExtrusionHeight()
   {
      return extrusionHeight;
   }
}
