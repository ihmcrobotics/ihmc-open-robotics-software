package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class GDXFootstepGraphic implements RenderableProvider
{
   public static final Color LEFT_FOOT_RED_COLOR = new Color(1.0f, 0.0f, 0.0f, 1.0f);
   public static final Color RIGHT_FOOT_GREEN_COLOR = new Color(0.0f, 0.5019608f, 0.0f, 1.0f);

   private final Color color;
   private final ConvexPolygon2D defaultContactPoints = new ConvexPolygon2D();

   public GDXFootstepGraphic(ArrayList<Point2D> controllerFootGroundContactPoints, Color color)
   {
      this.color = color;
      controllerFootGroundContactPoints.forEach(defaultContactPoints::addVertex);
      defaultContactPoints.update();
   }

   public void setTransparency(double opacity)
   {
      color.a = (float) opacity;
   }

   public void setColor(RobotSide side, Color color)
   {
      this.color.r = color.r;
      this.color.g = color.g;
      this.color.b = color.b;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   public void destroy()
   {

   }
}
