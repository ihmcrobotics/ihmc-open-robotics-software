package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.InclusionFunction;

/**
 * Created by agrabertilton on 1/15/15.
 */
public class BasicFootstepMask implements InclusionFunction<Point3D>
{
   private Point2D position = new Point2D();
   private double yaw;
   private double safetyBuffer;
   private boolean initialized = false;
   private ConvexPolygon2d footShapeWithBufferPolygon = new ConvexPolygon2d();

   public BasicFootstepMask(ContactablePlaneBody foot, double maskBufferSize)
   {
      this.safetyBuffer = maskBufferSize;

      footShapeWithBufferPolygon.clear();

      for (FramePoint2d vertex : foot.getContactPoints2d())
      {
         footShapeWithBufferPolygon.addVertex(inflate(vertex.getX()), inflate(vertex.getY()));
      }
      footShapeWithBufferPolygon.update();
   }

   public BasicFootstepMask(List<Point2D> footstepShape, double maskBufferSize)
   {
      this.safetyBuffer = maskBufferSize;

      footShapeWithBufferPolygon.clear();

      for (Point2D vertex : footstepShape)
      {
         footShapeWithBufferPolygon.addVertex(inflate(vertex.getX()), inflate(vertex.getY()));
      }
      footShapeWithBufferPolygon.update();
   }

   public BasicFootstepMask(ConvexPolygon2d footstepShape, double maskBufferSize)
   {
      int numVertices = footstepShape.getNumberOfVertices();
      Point2DReadOnly vertex;
      for (int i = 0; i < numVertices; i++)
      {
         vertex = footstepShape.getVertex(i);
         footShapeWithBufferPolygon.addVertex(inflate(vertex.getX()), inflate(vertex.getY()));
      }
      footShapeWithBufferPolygon.update();
   }

   private double inflate(double dimension)
   {
      return dimension + Math.signum(dimension) * safetyBuffer;
   }

   public void setPositionAndYaw(Point2D position, double yaw)
   {
      this.position.set(position.getX(), position.getY());
      this.yaw = yaw;
      initialized = true;
   }

   public void setPositionAndYaw(double x, double y, double yaw)
   {
      position.set(x,y);
      this.yaw = yaw;
      initialized = true;
   }

   public void setSafetyBuffer(double safetyBuffer)
   {
      this.safetyBuffer = safetyBuffer;
   }

   @Override
   public boolean isIncluded(Point3D inputPoint)
   {
      if (!initialized)
         return false;
      double x = inputPoint.getX() - position.getX();
      double y = inputPoint.getY() - position.getY();
      double cos = Math.cos(yaw);
      double sin = Math.sin(yaw);
      double xInFoot = x * cos + y * sin;
      double yInFoot = y * cos - x * sin;
      boolean isInside = footShapeWithBufferPolygon.isPointInside(xInFoot, yInFoot);

      return isInside;
   }
}
