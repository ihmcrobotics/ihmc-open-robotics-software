package us.ihmc.rdx.simulation.bullet;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.geometry.LineSegment3D;

public class RDXBulletPhysicsDebuggerLineSegment
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
