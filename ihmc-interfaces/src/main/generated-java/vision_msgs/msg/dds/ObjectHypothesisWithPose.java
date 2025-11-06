package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * An object hypothesis that contains pose information.
       * If you would like to define an array of ObjectHypothesisWithPose messages,
       * please see the Detection2D or Detection3D message types.
       */
public class ObjectHypothesisWithPose extends Packet<ObjectHypothesisWithPose> implements Settable<ObjectHypothesisWithPose>, EpsilonComparable<ObjectHypothesisWithPose>
{
   /**
            * The object hypothesis (ID and score).
            */
   public vision_msgs.msg.dds.ObjectHypothesis hypothesis_;
   /**
            * The 6D pose of the object hypothesis. This pose should be
            * defined as the pose of some fixed reference point on the object, such as
            * the geometric center of the bounding box, the center of mass of the
            * object or the origin of a reference mesh of the object.
            * Note that this pose is not stamped; frame information can be defined by
            * parent messages.
            * Also note that different classes predicted for the same input data may have
            * different predicted 6D poses.
            */
   public geometry_msgs.msg.dds.PoseWithCovariance pose_;

   public ObjectHypothesisWithPose()
   {
      hypothesis_ = new vision_msgs.msg.dds.ObjectHypothesis();
      pose_ = new geometry_msgs.msg.dds.PoseWithCovariance();
   }

   public ObjectHypothesisWithPose(ObjectHypothesisWithPose other)
   {
      this();
      set(other);
   }

   public void set(ObjectHypothesisWithPose other)
   {
      vision_msgs.msg.dds.ObjectHypothesisPubSubType.staticCopy(other.hypothesis_, hypothesis_);
      geometry_msgs.msg.dds.PoseWithCovariancePubSubType.staticCopy(other.pose_, pose_);
   }


   /**
            * The object hypothesis (ID and score).
            */
   public vision_msgs.msg.dds.ObjectHypothesis getHypothesis()
   {
      return hypothesis_;
   }


   /**
            * The 6D pose of the object hypothesis. This pose should be
            * defined as the pose of some fixed reference point on the object, such as
            * the geometric center of the bounding box, the center of mass of the
            * object or the origin of a reference mesh of the object.
            * Note that this pose is not stamped; frame information can be defined by
            * parent messages.
            * Also note that different classes predicted for the same input data may have
            * different predicted 6D poses.
            */
   public geometry_msgs.msg.dds.PoseWithCovariance getPose()
   {
      return pose_;
   }


   public static Supplier<ObjectHypothesisWithPosePubSubType> getPubSubType()
   {
      return ObjectHypothesisWithPosePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ObjectHypothesisWithPosePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ObjectHypothesisWithPose other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.hypothesis_.epsilonEquals(other.hypothesis_, epsilon)) return false;
      if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ObjectHypothesisWithPose)) return false;

      ObjectHypothesisWithPose otherMyClass = (ObjectHypothesisWithPose) other;

      if (!this.hypothesis_.equals(otherMyClass.hypothesis_)) return false;
      if (!this.pose_.equals(otherMyClass.pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ObjectHypothesisWithPose {");
      builder.append("hypothesis=");
      builder.append(this.hypothesis_);      builder.append(", ");
      builder.append("pose=");
      builder.append(this.pose_);
      builder.append("}");
      return builder.toString();
   }
}
