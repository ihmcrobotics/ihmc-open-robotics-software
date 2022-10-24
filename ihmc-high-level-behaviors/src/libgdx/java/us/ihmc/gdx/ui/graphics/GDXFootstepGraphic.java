package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

public class GDXFootstepGraphic implements RenderableProvider
{
   public static final Color LEFT_FOOT_RED_COLOR = new Color(1.0f, 0.0f, 0.0f, 1.0f);
   public static final Color RIGHT_FOOT_GREEN_COLOR = new Color(0.0f, 0.5019608f, 0.0f, 1.0f);
   public static final SideDependentList<Color> FOOT_COLORS = new SideDependentList<>(LEFT_FOOT_RED_COLOR, RIGHT_FOOT_GREEN_COLOR);
   private static final AtomicInteger INDEX = new AtomicInteger();

   private final Color color;
   private final ConvexPolygon2D defaultContactPoints = new ConvexPolygon2D();
   private ModelInstance modelInstance;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public GDXFootstepGraphic(SegmentDependentList<RobotSide, ArrayList<Point2D>> controllerFootGroundContactPoints, RobotSide side)
   {
      this(controllerFootGroundContactPoints.get(side), FOOT_COLORS.get(side));
   }

   public GDXFootstepGraphic(ArrayList<Point2D> controllerFootGroundContactPoints, Color color)
   {
      this.color = color;
      controllerFootGroundContactPoints.forEach(defaultContactPoints::addVertex);
      defaultContactPoints.update();
   }

   public void create()
   {
      Point3D[] vertices = new Point3D[defaultContactPoints.getNumberOfVertices()];
      for (int j = 0; j < vertices.length; j++)
      {
         vertices[j] = new Point3D(defaultContactPoints.getVertex(j).getX(),
                                   defaultContactPoints.getVertex(j).getY(),
                                   0.0);
      }

      modelInstance = GDXModelBuilder.buildModelInstance(meshBuilder ->
      {
         meshBuilder.addMultiLine(vertices, 0.01, color, true);
      }, "footstepGraphic" + INDEX.getAndIncrement());
   }

   public void setTransparency(double opacity)
   {
      color.a = (float) opacity; // TODO: Add blending mode attribute
   }

   public void setColor(Color color)
   {
      this.color.r = color.r;
      this.color.g = color.g;
      this.color.b = color.b;
   }

   public void setPose(Pose3DReadOnly pose)
   {
      GDXTools.toGDX(pose, tempTransform, modelInstance.transform);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public void destroy()
   {

   }
}
