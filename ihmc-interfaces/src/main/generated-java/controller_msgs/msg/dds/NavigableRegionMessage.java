package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class NavigableRegionMessage extends Packet<NavigableRegionMessage> implements Settable<NavigableRegionMessage>, EpsilonComparable<NavigableRegionMessage>
{
   public controller_msgs.msg.dds.PlanarRegionMessage home_region_;
   public controller_msgs.msg.dds.VisibilityClusterMessage home_region_cluster_;
   public controller_msgs.msg.dds.VisibilityMapMessage visibility_map_in_world_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityClusterMessage>  obstacle_clusters_;

   public NavigableRegionMessage()
   {
      home_region_ = new controller_msgs.msg.dds.PlanarRegionMessage();
      home_region_cluster_ = new controller_msgs.msg.dds.VisibilityClusterMessage();
      visibility_map_in_world_ = new controller_msgs.msg.dds.VisibilityMapMessage();
      obstacle_clusters_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityClusterMessage> (25, new controller_msgs.msg.dds.VisibilityClusterMessagePubSubType());

   }

   public NavigableRegionMessage(NavigableRegionMessage other)
   {
      this();
      set(other);
   }

   public void set(NavigableRegionMessage other)
   {
      controller_msgs.msg.dds.PlanarRegionMessagePubSubType.staticCopy(other.home_region_, home_region_);
      controller_msgs.msg.dds.VisibilityClusterMessagePubSubType.staticCopy(other.home_region_cluster_, home_region_cluster_);
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.staticCopy(other.visibility_map_in_world_, visibility_map_in_world_);
      obstacle_clusters_.set(other.obstacle_clusters_);
   }


   public controller_msgs.msg.dds.PlanarRegionMessage getHomeRegion()
   {
      return home_region_;
   }


   public controller_msgs.msg.dds.VisibilityClusterMessage getHomeRegionCluster()
   {
      return home_region_cluster_;
   }


   public controller_msgs.msg.dds.VisibilityMapMessage getVisibilityMapInWorld()
   {
      return visibility_map_in_world_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityClusterMessage>  getObstacleClusters()
   {
      return obstacle_clusters_;
   }


   public static Supplier<NavigableRegionMessagePubSubType> getPubSubType()
   {
      return NavigableRegionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return NavigableRegionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(NavigableRegionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.home_region_.epsilonEquals(other.home_region_, epsilon)) return false;
      if (!this.home_region_cluster_.epsilonEquals(other.home_region_cluster_, epsilon)) return false;
      if (!this.visibility_map_in_world_.epsilonEquals(other.visibility_map_in_world_, epsilon)) return false;
      if (this.obstacle_clusters_.size() != other.obstacle_clusters_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.obstacle_clusters_.size(); i++)
         {  if (!this.obstacle_clusters_.get(i).epsilonEquals(other.obstacle_clusters_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof NavigableRegionMessage)) return false;

      NavigableRegionMessage otherMyClass = (NavigableRegionMessage) other;

      if (!this.home_region_.equals(otherMyClass.home_region_)) return false;
      if (!this.home_region_cluster_.equals(otherMyClass.home_region_cluster_)) return false;
      if (!this.visibility_map_in_world_.equals(otherMyClass.visibility_map_in_world_)) return false;
      if (!this.obstacle_clusters_.equals(otherMyClass.obstacle_clusters_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("NavigableRegionMessage {");
      builder.append("home_region=");
      builder.append(this.home_region_);      builder.append(", ");
      builder.append("home_region_cluster=");
      builder.append(this.home_region_cluster_);      builder.append(", ");
      builder.append("visibility_map_in_world=");
      builder.append(this.visibility_map_in_world_);      builder.append(", ");
      builder.append("obstacle_clusters=");
      builder.append(this.obstacle_clusters_);
      builder.append("}");
      return builder.toString();
   }
}
