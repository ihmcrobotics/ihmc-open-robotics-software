package us.ihmc.euclid.jros2.messages;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.fastddsjava.cdr.CDRBuffer;
import us.ihmc.jros2.ROS2Message;

/**
 * Custom ROS2Message wrapper that adapts Euclid Point3D to geometry_msgs/Point.
 * This allows seamless use of Euclid geometry types with ROS2 messaging.
 */
public class EuclidPoint3DMessage implements ROS2Message<EuclidPoint3DMessage>
{
   /**
    * ROS 2 type name - REQUIRED by jros2
    */
   public static final String name = "geometry_msgs::msg::dds_::Point_";

   private final Point3D point;

   /**
    * Default constructor - REQUIRED by jros2
    */
   public EuclidPoint3DMessage()
   {
      this.point = new Point3D();
   }

   /**
    * Constructor that wraps an existing Point3D.
    * Note: This does NOT copy the point, it wraps the same instance.
    */
   public EuclidPoint3DMessage(Point3D point)
   {
      this.point = point;
   }

   @Override
   public int calculateSizeBytes(int currentAlignment)
   {
      int initialAlignment = currentAlignment;

      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // x
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // y
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // z

      return currentAlignment - initialAlignment;
   }

   @Override
   public void serialize(CDRBuffer buffer)
   {
      buffer.writeDouble(point.getX());
      buffer.writeDouble(point.getY());
      buffer.writeDouble(point.getZ());
   }

   @Override
   public void deserialize(CDRBuffer buffer)
   {
      point.setX(buffer.readDouble());
      point.setY(buffer.readDouble());
      point.setZ(buffer.readDouble());
   }

   @Override
   public void set(EuclidPoint3DMessage from)
   {
      this.point.set(from.point);
   }

   /**
    * Set from a Point3D directly.
    */
   public void set(Point3D from)
   {
      this.point.set(from);
   }

   /**
    * Get the wrapped Euclid Point3D object.
    * This returns the actual object, not a copy.
    */
   public Point3D getPoint()
   {
      return point;
   }
}
