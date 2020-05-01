package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class UIPositionCheckerPacket extends Packet<UIPositionCheckerPacket> implements Settable<UIPositionCheckerPacket>, EpsilonComparable<UIPositionCheckerPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.euclid.tuple3D.Point3D position_;

   public us.ihmc.euclid.tuple4D.Quaternion orientation_;

   public UIPositionCheckerPacket()
   {


      position_ = new us.ihmc.euclid.tuple3D.Point3D();

      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();

   }

   public UIPositionCheckerPacket(UIPositionCheckerPacket other)
   {
      this();
      set(other);
   }

   public void set(UIPositionCheckerPacket other)
   {

      sequence_id_ = other.sequence_id_;


      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }



   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }



   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }


   public static Supplier<UIPositionCheckerPacketPubSubType> getPubSubType()
   {
      return UIPositionCheckerPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return UIPositionCheckerPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(UIPositionCheckerPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;

      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof UIPositionCheckerPacket)) return false;

      UIPositionCheckerPacket otherMyClass = (UIPositionCheckerPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.position_.equals(otherMyClass.position_)) return false;

      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UIPositionCheckerPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");

      builder.append("orientation=");
      builder.append(this.orientation_);
      builder.append("}");
      return builder.toString();
   }
}
