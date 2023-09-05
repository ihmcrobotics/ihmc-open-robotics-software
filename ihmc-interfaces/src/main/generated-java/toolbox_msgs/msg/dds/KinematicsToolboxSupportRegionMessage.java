package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       * The inverse kinematics solver provides the option to constrain the projection onto the horizontal plane of the center-of-mass
       * to remain inside the given support polygon.
       * This allows to constrain the solution to be statically stable.
       * While the support polygon is usually determined based on the active controller, i.e. via CapturabilityBasedStatus or MultiContactBalanceStatus,
       * this message can be used to directly specify it.
       */
public class KinematicsToolboxSupportRegionMessage extends Packet<KinematicsToolboxSupportRegionMessage> implements Settable<KinematicsToolboxSupportRegionMessage>, EpsilonComparable<KinematicsToolboxSupportRegionMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies the minimum distance to keep the center of mass away from the support polygon's edges.
            */
   public double center_of_mass_margin_ = -1.0;
   /**
            * The list of vertices of the support region.
            * These are nominally identical to the robot's contact points, but might be different when a hand-hold is present, for example.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  support_region_vertices_;
   /**
            * The frame id of the vertex. If empty, world frame is assumed.
            */
   public us.ihmc.idl.IDLSequence.Long  support_region_vertex_frames_;

   public KinematicsToolboxSupportRegionMessage()
   {
      support_region_vertices_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (100, new geometry_msgs.msg.dds.PointPubSubType());
      support_region_vertex_frames_ = new us.ihmc.idl.IDLSequence.Long (100, "type_11");


   }

   public KinematicsToolboxSupportRegionMessage(KinematicsToolboxSupportRegionMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxSupportRegionMessage other)
   {
      sequence_id_ = other.sequence_id_;

      center_of_mass_margin_ = other.center_of_mass_margin_;

      support_region_vertices_.set(other.support_region_vertices_);
      support_region_vertex_frames_.set(other.support_region_vertex_frames_);
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

   /**
            * Specifies the minimum distance to keep the center of mass away from the support polygon's edges.
            */
   public void setCenterOfMassMargin(double center_of_mass_margin)
   {
      center_of_mass_margin_ = center_of_mass_margin;
   }
   /**
            * Specifies the minimum distance to keep the center of mass away from the support polygon's edges.
            */
   public double getCenterOfMassMargin()
   {
      return center_of_mass_margin_;
   }


   /**
            * The list of vertices of the support region.
            * These are nominally identical to the robot's contact points, but might be different when a hand-hold is present, for example.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getSupportRegionVertices()
   {
      return support_region_vertices_;
   }


   /**
            * The frame id of the vertex. If empty, world frame is assumed.
            */
   public us.ihmc.idl.IDLSequence.Long  getSupportRegionVertexFrames()
   {
      return support_region_vertex_frames_;
   }


   public static Supplier<KinematicsToolboxSupportRegionMessagePubSubType> getPubSubType()
   {
      return KinematicsToolboxSupportRegionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxSupportRegionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxSupportRegionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_of_mass_margin_, other.center_of_mass_margin_, epsilon)) return false;

      if (this.support_region_vertices_.size() != other.support_region_vertices_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.support_region_vertices_.size(); i++)
         {  if (!this.support_region_vertices_.get(i).epsilonEquals(other.support_region_vertices_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.support_region_vertex_frames_, other.support_region_vertex_frames_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxSupportRegionMessage)) return false;

      KinematicsToolboxSupportRegionMessage otherMyClass = (KinematicsToolboxSupportRegionMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.center_of_mass_margin_ != otherMyClass.center_of_mass_margin_) return false;

      if (!this.support_region_vertices_.equals(otherMyClass.support_region_vertices_)) return false;
      if (!this.support_region_vertex_frames_.equals(otherMyClass.support_region_vertex_frames_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxSupportRegionMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("center_of_mass_margin=");
      builder.append(this.center_of_mass_margin_);      builder.append(", ");
      builder.append("support_region_vertices=");
      builder.append(this.support_region_vertices_);      builder.append(", ");
      builder.append("support_region_vertex_frames=");
      builder.append(this.support_region_vertex_frames_);
      builder.append("}");
      return builder.toString();
   }
}
