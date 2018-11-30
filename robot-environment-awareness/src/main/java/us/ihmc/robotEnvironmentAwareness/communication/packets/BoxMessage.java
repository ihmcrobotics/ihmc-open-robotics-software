package us.ihmc.robotEnvironmentAwareness.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;

public class BoxMessage extends Packet<BoxMessage>
{
   public boolean isEmpty;
   public Vector3D32 size;
   public Point3D32 center;
   public Quaternion32 orientation;

   public BoxMessage()
   {
   }

   public static BoxMessage emptyBox()
   {
      BoxMessage empty = new BoxMessage();
      empty.isEmpty = true;
      empty.size = null;
      empty.center = null;
      empty.orientation = null;
      return empty;
   }

   @Override
   public void set(BoxMessage other)
   {
      setPacketInformation(other);
      isEmpty = other.isEmpty;
      size = new Vector3D32(other.size);
      center = new Point3D32(center);
      orientation = new Quaternion32(other.orientation);
   }

   public boolean isEmpty()
   {
      return isEmpty;
   }

   public void setSize(Vector3D size)
   {
      isEmpty = false;
      this.size = new Vector3D32(size);
   }

   public void setSize(Vector3D32 size)
   {
      isEmpty = false;
      this.size = size;
   }

   public void setCenter(Point3D center)
   {
      isEmpty = false;
      this.center = new Point3D32(center);
   }

   public void setCenter(Point3D32 center)
   {
      isEmpty = false;
      this.center = center;
   }

   public void setOrientation(Quaternion orientation)
   {
      isEmpty = false;
      this.orientation = new Quaternion32(orientation);
   }

   public void setOrientation(Quaternion32 orientation)
   {
      isEmpty = false;
      this.orientation = orientation;
   }

   public Vector3D32 getSize()
   {
      return size;
   }

   public Point3D32 getCenter()
   {
      return center;
   }

   public Quaternion32 getOrientation()
   {
      return orientation;
   }

   @Override
   public boolean epsilonEquals(BoxMessage other, double epsilon)
   {
      if (!size.epsilonEquals(other.size, (float) epsilon))
         return false;
      if (!center.epsilonEquals(other.center, (float) epsilon))
         return false;
      if (!orientation.epsilonEquals(other.orientation, (float) epsilon))
         return false;
      return true;
   }
}
