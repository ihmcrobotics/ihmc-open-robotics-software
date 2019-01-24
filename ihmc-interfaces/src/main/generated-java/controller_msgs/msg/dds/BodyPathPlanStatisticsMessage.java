package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class BodyPathPlanStatisticsMessage extends Packet<BodyPathPlanStatisticsMessage> implements Settable<BodyPathPlanStatisticsMessage>, EpsilonComparable<BodyPathPlanStatisticsMessage>
{
   public static final int NO_PLAN_ID = -1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public int plan_id_ = -1;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage>  navigable_regions_;
   public controller_msgs.msg.dds.VisibilityMapMessage inter_regions_map_;
   public controller_msgs.msg.dds.VisibilityMapMessage start_visibility_map_;
   public controller_msgs.msg.dds.VisibilityMapMessage goal_visibility_map_;

   public BodyPathPlanStatisticsMessage()
   {
      navigable_regions_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage> (25, new controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessagePubSubType());
      inter_regions_map_ = new controller_msgs.msg.dds.VisibilityMapMessage();
      start_visibility_map_ = new controller_msgs.msg.dds.VisibilityMapMessage();
      goal_visibility_map_ = new controller_msgs.msg.dds.VisibilityMapMessage();

   }

   public BodyPathPlanStatisticsMessage(BodyPathPlanStatisticsMessage other)
   {
      this();
      set(other);
   }

   public void set(BodyPathPlanStatisticsMessage other)
   {
      sequence_id_ = other.sequence_id_;

      plan_id_ = other.plan_id_;

      navigable_regions_.set(other.navigable_regions_);
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.staticCopy(other.inter_regions_map_, inter_regions_map_);
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.staticCopy(other.start_visibility_map_, start_visibility_map_);
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.staticCopy(other.goal_visibility_map_, goal_visibility_map_);
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

   public void setPlanId(int plan_id)
   {
      plan_id_ = plan_id;
   }
   public int getPlanId()
   {
      return plan_id_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage>  getNavigableRegions()
   {
      return navigable_regions_;
   }


   public controller_msgs.msg.dds.VisibilityMapMessage getInterRegionsMap()
   {
      return inter_regions_map_;
   }


   public controller_msgs.msg.dds.VisibilityMapMessage getStartVisibilityMap()
   {
      return start_visibility_map_;
   }


   public controller_msgs.msg.dds.VisibilityMapMessage getGoalVisibilityMap()
   {
      return goal_visibility_map_;
   }


   public static Supplier<BodyPathPlanStatisticsMessagePubSubType> getPubSubType()
   {
      return BodyPathPlanStatisticsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BodyPathPlanStatisticsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BodyPathPlanStatisticsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.plan_id_, other.plan_id_, epsilon)) return false;

      if (this.navigable_regions_.size() != other.navigable_regions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.navigable_regions_.size(); i++)
         {  if (!this.navigable_regions_.get(i).epsilonEquals(other.navigable_regions_.get(i), epsilon)) return false; }
      }

      if (!this.inter_regions_map_.epsilonEquals(other.inter_regions_map_, epsilon)) return false;
      if (!this.start_visibility_map_.epsilonEquals(other.start_visibility_map_, epsilon)) return false;
      if (!this.goal_visibility_map_.epsilonEquals(other.goal_visibility_map_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BodyPathPlanStatisticsMessage)) return false;

      BodyPathPlanStatisticsMessage otherMyClass = (BodyPathPlanStatisticsMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.plan_id_ != otherMyClass.plan_id_) return false;

      if (!this.navigable_regions_.equals(otherMyClass.navigable_regions_)) return false;
      if (!this.inter_regions_map_.equals(otherMyClass.inter_regions_map_)) return false;
      if (!this.start_visibility_map_.equals(otherMyClass.start_visibility_map_)) return false;
      if (!this.goal_visibility_map_.equals(otherMyClass.goal_visibility_map_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BodyPathPlanStatisticsMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("plan_id=");
      builder.append(this.plan_id_);      builder.append(", ");
      builder.append("navigable_regions=");
      builder.append(this.navigable_regions_);      builder.append(", ");
      builder.append("inter_regions_map=");
      builder.append(this.inter_regions_map_);      builder.append(", ");
      builder.append("start_visibility_map=");
      builder.append(this.start_visibility_map_);      builder.append(", ");
      builder.append("goal_visibility_map=");
      builder.append(this.goal_visibility_map_);
      builder.append("}");
      return builder.toString();
   }
}
