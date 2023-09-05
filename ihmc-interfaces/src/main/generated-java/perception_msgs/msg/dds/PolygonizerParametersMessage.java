package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC Robot Environment Awareness (REA) module.
       */
public class PolygonizerParametersMessage extends Packet<PolygonizerParametersMessage> implements Settable<PolygonizerParametersMessage>, EpsilonComparable<PolygonizerParametersMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Threshold used when creating a new concave hull.
            * Uses the Duckham and al. (2008) algorithm defined in the paper "Efficient generation of
            * simple polygons for characterizing the shape of a set of points in the plane".
            */
   public double concave_hull_threshold_ = 0.15;
   /**
            * The minimum number of nodes required for a region to be polygonized.
            */
   public int min_number_of_nodes_ = 10;
   /**
            * Filter parameter on the concave hull of a region. Used to removed vertices describing shallow angle.
            */
   public double shallow_angle_threshold_ = 0.01745;
   /**
            * Filter parameter on the concave hull of a region. Used to removed vertices that create peaks.
            */
   public double peak_angle_threshold_ = 2.967;
   /**
            * Filter parameter on the concave hull of a region. Used to removed short edges.
            */
   public double length_threshold_ = 0.05;
   /**
            * Threshold used for decomposing the concave hull into convex polygons. Describes the maximum depth
            * of a concavity before the concave hull gets split in 2.
            */
   public double depth_threshold_ = 0.1;
   /**
            * Filter for splitting concave hulls at any narrow passage which width is less than (2 * length_threshold).
            */
   public boolean cut_narrow_passage_ = true;

   public PolygonizerParametersMessage()
   {
   }

   public PolygonizerParametersMessage(PolygonizerParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(PolygonizerParametersMessage other)
   {
      sequence_id_ = other.sequence_id_;

      concave_hull_threshold_ = other.concave_hull_threshold_;

      min_number_of_nodes_ = other.min_number_of_nodes_;

      shallow_angle_threshold_ = other.shallow_angle_threshold_;

      peak_angle_threshold_ = other.peak_angle_threshold_;

      length_threshold_ = other.length_threshold_;

      depth_threshold_ = other.depth_threshold_;

      cut_narrow_passage_ = other.cut_narrow_passage_;

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
            * Threshold used when creating a new concave hull.
            * Uses the Duckham and al. (2008) algorithm defined in the paper "Efficient generation of
            * simple polygons for characterizing the shape of a set of points in the plane".
            */
   public void setConcaveHullThreshold(double concave_hull_threshold)
   {
      concave_hull_threshold_ = concave_hull_threshold;
   }
   /**
            * Threshold used when creating a new concave hull.
            * Uses the Duckham and al. (2008) algorithm defined in the paper "Efficient generation of
            * simple polygons for characterizing the shape of a set of points in the plane".
            */
   public double getConcaveHullThreshold()
   {
      return concave_hull_threshold_;
   }

   /**
            * The minimum number of nodes required for a region to be polygonized.
            */
   public void setMinNumberOfNodes(int min_number_of_nodes)
   {
      min_number_of_nodes_ = min_number_of_nodes;
   }
   /**
            * The minimum number of nodes required for a region to be polygonized.
            */
   public int getMinNumberOfNodes()
   {
      return min_number_of_nodes_;
   }

   /**
            * Filter parameter on the concave hull of a region. Used to removed vertices describing shallow angle.
            */
   public void setShallowAngleThreshold(double shallow_angle_threshold)
   {
      shallow_angle_threshold_ = shallow_angle_threshold;
   }
   /**
            * Filter parameter on the concave hull of a region. Used to removed vertices describing shallow angle.
            */
   public double getShallowAngleThreshold()
   {
      return shallow_angle_threshold_;
   }

   /**
            * Filter parameter on the concave hull of a region. Used to removed vertices that create peaks.
            */
   public void setPeakAngleThreshold(double peak_angle_threshold)
   {
      peak_angle_threshold_ = peak_angle_threshold;
   }
   /**
            * Filter parameter on the concave hull of a region. Used to removed vertices that create peaks.
            */
   public double getPeakAngleThreshold()
   {
      return peak_angle_threshold_;
   }

   /**
            * Filter parameter on the concave hull of a region. Used to removed short edges.
            */
   public void setLengthThreshold(double length_threshold)
   {
      length_threshold_ = length_threshold;
   }
   /**
            * Filter parameter on the concave hull of a region. Used to removed short edges.
            */
   public double getLengthThreshold()
   {
      return length_threshold_;
   }

   /**
            * Threshold used for decomposing the concave hull into convex polygons. Describes the maximum depth
            * of a concavity before the concave hull gets split in 2.
            */
   public void setDepthThreshold(double depth_threshold)
   {
      depth_threshold_ = depth_threshold;
   }
   /**
            * Threshold used for decomposing the concave hull into convex polygons. Describes the maximum depth
            * of a concavity before the concave hull gets split in 2.
            */
   public double getDepthThreshold()
   {
      return depth_threshold_;
   }

   /**
            * Filter for splitting concave hulls at any narrow passage which width is less than (2 * length_threshold).
            */
   public void setCutNarrowPassage(boolean cut_narrow_passage)
   {
      cut_narrow_passage_ = cut_narrow_passage;
   }
   /**
            * Filter for splitting concave hulls at any narrow passage which width is less than (2 * length_threshold).
            */
   public boolean getCutNarrowPassage()
   {
      return cut_narrow_passage_;
   }


   public static Supplier<PolygonizerParametersMessagePubSubType> getPubSubType()
   {
      return PolygonizerParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PolygonizerParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PolygonizerParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.concave_hull_threshold_, other.concave_hull_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_number_of_nodes_, other.min_number_of_nodes_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shallow_angle_threshold_, other.shallow_angle_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.peak_angle_threshold_, other.peak_angle_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.length_threshold_, other.length_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.depth_threshold_, other.depth_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.cut_narrow_passage_, other.cut_narrow_passage_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PolygonizerParametersMessage)) return false;

      PolygonizerParametersMessage otherMyClass = (PolygonizerParametersMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.concave_hull_threshold_ != otherMyClass.concave_hull_threshold_) return false;

      if(this.min_number_of_nodes_ != otherMyClass.min_number_of_nodes_) return false;

      if(this.shallow_angle_threshold_ != otherMyClass.shallow_angle_threshold_) return false;

      if(this.peak_angle_threshold_ != otherMyClass.peak_angle_threshold_) return false;

      if(this.length_threshold_ != otherMyClass.length_threshold_) return false;

      if(this.depth_threshold_ != otherMyClass.depth_threshold_) return false;

      if(this.cut_narrow_passage_ != otherMyClass.cut_narrow_passage_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PolygonizerParametersMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("concave_hull_threshold=");
      builder.append(this.concave_hull_threshold_);      builder.append(", ");
      builder.append("min_number_of_nodes=");
      builder.append(this.min_number_of_nodes_);      builder.append(", ");
      builder.append("shallow_angle_threshold=");
      builder.append(this.shallow_angle_threshold_);      builder.append(", ");
      builder.append("peak_angle_threshold=");
      builder.append(this.peak_angle_threshold_);      builder.append(", ");
      builder.append("length_threshold=");
      builder.append(this.length_threshold_);      builder.append(", ");
      builder.append("depth_threshold=");
      builder.append(this.depth_threshold_);      builder.append(", ");
      builder.append("cut_narrow_passage=");
      builder.append(this.cut_narrow_passage_);
      builder.append("}");
      return builder.toString();
   }
}
