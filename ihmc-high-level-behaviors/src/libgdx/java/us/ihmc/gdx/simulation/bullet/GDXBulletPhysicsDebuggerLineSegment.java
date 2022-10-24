package us.ihmc.gdx.simulation.bullet;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.geometry.LineSegment3D;

public class GDXBulletPhysicsDebuggerLineSegment
{
   private final LineSegment3D lineSegment = new LineSegment3D();
   private final Color color = new Color();

   public LineSegment3D getLineSegment()
   {
      return lineSegment;
   }

   public Color getColor()
   {
      return color;
   }
}
