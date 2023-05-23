package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PoseListMessage extends Packet<PoseListMessage> implements Settable<PoseListMessage>, EpsilonComparable<PoseListMessage>
{
   /**
            * A list of poses
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  poses_;

   public PoseListMessage()
   {
      poses_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (100, new geometry_msgs.msg.dds.PosePubSubType());

   }

   public PoseListMessage(PoseListMessage other)
   {
      this();
      set(other);
   }

   public void set(PoseListMessage other)
   {
      poses_.set(other.poses_);
   }


   /**
            * A list of poses
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getPoses()
   {
      return poses_;
   }


   public static Supplier<PoseListMessagePubSubType> getPubSubType()
   {
      return PoseListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PoseListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PoseListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.poses_.size() != other.poses_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.poses_.size(); i++)
         {  if (!this.poses_.get(i).epsilonEquals(other.poses_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PoseListMessage)) return false;

      PoseListMessage otherMyClass = (PoseListMessage) other;

      if (!this.poses_.equals(otherMyClass.poses_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PoseListMessage {");
      builder.append("poses=");
      builder.append(this.poses_);
      builder.append("}");
      return builder.toString();
   }
}
