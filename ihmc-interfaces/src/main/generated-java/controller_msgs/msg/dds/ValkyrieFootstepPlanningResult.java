package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ValkyrieFootstepPlanningResult extends Packet<ValkyrieFootstepPlanningResult> implements Settable<ValkyrieFootstepPlanningResult>, EpsilonComparable<ValkyrieFootstepPlanningResult>
{
   public int plan_id_ = -1;
   public controller_msgs.msg.dds.FootstepDataListMessage footstep_data_list_;
   public controller_msgs.msg.dds.PlanarRegionsListMessage planar_regions_list_;

   public ValkyrieFootstepPlanningResult()
   {
      footstep_data_list_ = new controller_msgs.msg.dds.FootstepDataListMessage();
      planar_regions_list_ = new controller_msgs.msg.dds.PlanarRegionsListMessage();
   }

   public ValkyrieFootstepPlanningResult(ValkyrieFootstepPlanningResult other)
   {
      this();
      set(other);
   }

   public void set(ValkyrieFootstepPlanningResult other)
   {
      plan_id_ = other.plan_id_;

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.staticCopy(other.footstep_data_list_, footstep_data_list_);
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_list_, planar_regions_list_);
   }

   public void setPlanId(int plan_id)
   {
      plan_id_ = plan_id;
   }
   public int getPlanId()
   {
      return plan_id_;
   }


   public controller_msgs.msg.dds.FootstepDataListMessage getFootstepDataList()
   {
      return footstep_data_list_;
   }


   public controller_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegionsList()
   {
      return planar_regions_list_;
   }


   public static Supplier<ValkyrieFootstepPlanningResultPubSubType> getPubSubType()
   {
      return ValkyrieFootstepPlanningResultPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ValkyrieFootstepPlanningResultPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ValkyrieFootstepPlanningResult other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.plan_id_, other.plan_id_, epsilon)) return false;

      if (!this.footstep_data_list_.epsilonEquals(other.footstep_data_list_, epsilon)) return false;
      if (!this.planar_regions_list_.epsilonEquals(other.planar_regions_list_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ValkyrieFootstepPlanningResult)) return false;

      ValkyrieFootstepPlanningResult otherMyClass = (ValkyrieFootstepPlanningResult) other;

      if(this.plan_id_ != otherMyClass.plan_id_) return false;

      if (!this.footstep_data_list_.equals(otherMyClass.footstep_data_list_)) return false;
      if (!this.planar_regions_list_.equals(otherMyClass.planar_regions_list_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ValkyrieFootstepPlanningResult {");
      builder.append("plan_id=");
      builder.append(this.plan_id_);      builder.append(", ");
      builder.append("footstep_data_list=");
      builder.append(this.footstep_data_list_);      builder.append(", ");
      builder.append("planar_regions_list=");
      builder.append(this.planar_regions_list_);
      builder.append("}");
      return builder.toString();
   }
}
