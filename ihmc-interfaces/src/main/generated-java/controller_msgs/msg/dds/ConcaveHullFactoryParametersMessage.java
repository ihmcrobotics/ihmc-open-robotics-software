package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * REA parameters used to create concave hull of region
       */
public class ConcaveHullFactoryParametersMessage extends Packet<ConcaveHullFactoryParametersMessage> implements Settable<ConcaveHullFactoryParametersMessage>, EpsilonComparable<ConcaveHullFactoryParametersMessage>
{
   public double edge_length_threshold_ = 0.1;
   public boolean remove_all_triangles_with_two_border_edges_ = true;
   public boolean allow_splitting_concave_hull_ = true;
   public int max_number_of_iterations_ = 5000;
   public double triangulation_tolerance_;

   public ConcaveHullFactoryParametersMessage()
   {
   }

   public ConcaveHullFactoryParametersMessage(ConcaveHullFactoryParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(ConcaveHullFactoryParametersMessage other)
   {
      edge_length_threshold_ = other.edge_length_threshold_;

      remove_all_triangles_with_two_border_edges_ = other.remove_all_triangles_with_two_border_edges_;

      allow_splitting_concave_hull_ = other.allow_splitting_concave_hull_;

      max_number_of_iterations_ = other.max_number_of_iterations_;

      triangulation_tolerance_ = other.triangulation_tolerance_;

   }

   public void setEdgeLengthThreshold(double edge_length_threshold)
   {
      edge_length_threshold_ = edge_length_threshold;
   }
   public double getEdgeLengthThreshold()
   {
      return edge_length_threshold_;
   }

   public void setRemoveAllTrianglesWithTwoBorderEdges(boolean remove_all_triangles_with_two_border_edges)
   {
      remove_all_triangles_with_two_border_edges_ = remove_all_triangles_with_two_border_edges;
   }
   public boolean getRemoveAllTrianglesWithTwoBorderEdges()
   {
      return remove_all_triangles_with_two_border_edges_;
   }

   public void setAllowSplittingConcaveHull(boolean allow_splitting_concave_hull)
   {
      allow_splitting_concave_hull_ = allow_splitting_concave_hull;
   }
   public boolean getAllowSplittingConcaveHull()
   {
      return allow_splitting_concave_hull_;
   }

   public void setMaxNumberOfIterations(int max_number_of_iterations)
   {
      max_number_of_iterations_ = max_number_of_iterations;
   }
   public int getMaxNumberOfIterations()
   {
      return max_number_of_iterations_;
   }

   public void setTriangulationTolerance(double triangulation_tolerance)
   {
      triangulation_tolerance_ = triangulation_tolerance;
   }
   public double getTriangulationTolerance()
   {
      return triangulation_tolerance_;
   }


   public static Supplier<ConcaveHullFactoryParametersMessagePubSubType> getPubSubType()
   {
      return ConcaveHullFactoryParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ConcaveHullFactoryParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ConcaveHullFactoryParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.edge_length_threshold_, other.edge_length_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.remove_all_triangles_with_two_border_edges_, other.remove_all_triangles_with_two_border_edges_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.allow_splitting_concave_hull_, other.allow_splitting_concave_hull_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_number_of_iterations_, other.max_number_of_iterations_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.triangulation_tolerance_, other.triangulation_tolerance_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ConcaveHullFactoryParametersMessage)) return false;

      ConcaveHullFactoryParametersMessage otherMyClass = (ConcaveHullFactoryParametersMessage) other;

      if(this.edge_length_threshold_ != otherMyClass.edge_length_threshold_) return false;

      if(this.remove_all_triangles_with_two_border_edges_ != otherMyClass.remove_all_triangles_with_two_border_edges_) return false;

      if(this.allow_splitting_concave_hull_ != otherMyClass.allow_splitting_concave_hull_) return false;

      if(this.max_number_of_iterations_ != otherMyClass.max_number_of_iterations_) return false;

      if(this.triangulation_tolerance_ != otherMyClass.triangulation_tolerance_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConcaveHullFactoryParametersMessage {");
      builder.append("edge_length_threshold=");
      builder.append(this.edge_length_threshold_);      builder.append(", ");
      builder.append("remove_all_triangles_with_two_border_edges=");
      builder.append(this.remove_all_triangles_with_two_border_edges_);      builder.append(", ");
      builder.append("allow_splitting_concave_hull=");
      builder.append(this.allow_splitting_concave_hull_);      builder.append(", ");
      builder.append("max_number_of_iterations=");
      builder.append(this.max_number_of_iterations_);      builder.append(", ");
      builder.append("triangulation_tolerance=");
      builder.append(this.triangulation_tolerance_);
      builder.append("}");
      return builder.toString();
   }
}
