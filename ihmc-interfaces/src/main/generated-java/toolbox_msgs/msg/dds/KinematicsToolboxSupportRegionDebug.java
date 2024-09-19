package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KinematicsToolboxSupportRegionDebug extends Packet<KinematicsToolboxSupportRegionDebug> implements Settable<KinematicsToolboxSupportRegionDebug>, EpsilonComparable<KinematicsToolboxSupportRegionDebug>
{
   /**
            * Multi-contact feasible com region computed by the toolbox (if upper body is load-bearing)
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  multi_contact_feasible_com_region_;
   /**
            * CoM stability margin if upper body is load-bearing
            */
   public double center_of_mass_stability_margin_ = -1.0;

   public KinematicsToolboxSupportRegionDebug()
   {
      multi_contact_feasible_com_region_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (18, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public KinematicsToolboxSupportRegionDebug(KinematicsToolboxSupportRegionDebug other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxSupportRegionDebug other)
   {
      multi_contact_feasible_com_region_.set(other.multi_contact_feasible_com_region_);
      center_of_mass_stability_margin_ = other.center_of_mass_stability_margin_;

   }


   /**
            * Multi-contact feasible com region computed by the toolbox (if upper body is load-bearing)
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getMultiContactFeasibleComRegion()
   {
      return multi_contact_feasible_com_region_;
   }

   /**
            * CoM stability margin if upper body is load-bearing
            */
   public void setCenterOfMassStabilityMargin(double center_of_mass_stability_margin)
   {
      center_of_mass_stability_margin_ = center_of_mass_stability_margin;
   }
   /**
            * CoM stability margin if upper body is load-bearing
            */
   public double getCenterOfMassStabilityMargin()
   {
      return center_of_mass_stability_margin_;
   }


   public static Supplier<KinematicsToolboxSupportRegionDebugPubSubType> getPubSubType()
   {
      return KinematicsToolboxSupportRegionDebugPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxSupportRegionDebugPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxSupportRegionDebug other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.multi_contact_feasible_com_region_.size() != other.multi_contact_feasible_com_region_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.multi_contact_feasible_com_region_.size(); i++)
         {  if (!this.multi_contact_feasible_com_region_.get(i).epsilonEquals(other.multi_contact_feasible_com_region_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_of_mass_stability_margin_, other.center_of_mass_stability_margin_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxSupportRegionDebug)) return false;

      KinematicsToolboxSupportRegionDebug otherMyClass = (KinematicsToolboxSupportRegionDebug) other;

      if (!this.multi_contact_feasible_com_region_.equals(otherMyClass.multi_contact_feasible_com_region_)) return false;
      if(this.center_of_mass_stability_margin_ != otherMyClass.center_of_mass_stability_margin_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxSupportRegionDebug {");
      builder.append("multi_contact_feasible_com_region=");
      builder.append(this.multi_contact_feasible_com_region_);      builder.append(", ");
      builder.append("center_of_mass_stability_margin=");
      builder.append(this.center_of_mass_stability_margin_);
      builder.append("}");
      return builder.toString();
   }
}
