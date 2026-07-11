package us.ihmc.euclid.jros2.messages;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.fastddsjava.cdr.CDRBuffer;
import us.ihmc.jros2.ROS2Message;

/**
 * Custom ROS2Message wrapper that adapts Euclid Vector3D to geometry_msgs/Vector3.
 * This allows seamless use of Euclid geometry types with ROS2 messaging.
 */
public class EuclidVector3DMessage implements ROS2Message<EuclidVector3DMessage>
{
   /**
    * ROS 2 type name - REQUIRED by jros2
    */
   public static final String name = "geometry_msgs::msg::dds_::Vector3_";

   private final Vector3D vector;

   /**
    * Default constructor - REQUIRED by jros2
    */
   public EuclidVector3DMessage()
   {
      this.vector = new Vector3D();
   }

   /**
    * Constructor that wraps an existing Vector3D.
    * Note: This does NOT copy the vector, it wraps the same instance.
    */
   public EuclidVector3DMessage(Vector3D vector)
   {
      this.vector = vector;
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
      buffer.writeDouble(vector.getX());
      buffer.writeDouble(vector.getY());
      buffer.writeDouble(vector.getZ());
   }

   @Override
   public void deserialize(CDRBuffer buffer)
   {
      vector.setX(buffer.readDouble());
      vector.setY(buffer.readDouble());
      vector.setZ(buffer.readDouble());
   }

   @Override
   public void set(EuclidVector3DMessage from)
   {
      this.vector.set(from.vector);
   }

   public void set(Vector2D from)
   {
      this.vector.set(from);
   }

   public void set(Vector2DBasics from)
   {
      this.vector.set(from);
   }

   public void set(Vector2DReadOnly from)
   {
      this.vector.set(from);
   }

   /**
    * Set from a Vector3DBasics directly.
    */
   public void set(Vector3D from)
   {
      this.vector.set(from);
   }

   /**
    * Set from a Vector3DBasics directly.
    */
   public void set(Vector3DBasics from)
   {
      this.vector.set(from);
   }

   /**
    * Set from a Vector3DReadOnly directly.
    */
   public void set(Vector3DReadOnly from)
   {
      this.vector.set(from);
   }

   /**
    * Get the wrapped Euclid Vector3D object.
    * This returns the actual object, not a copy.
    */
   public Vector3D getVector()
   {
      return vector;
   }
}
