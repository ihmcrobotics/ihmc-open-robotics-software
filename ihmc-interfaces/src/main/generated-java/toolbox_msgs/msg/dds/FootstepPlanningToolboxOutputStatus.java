package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class FootstepPlanningToolboxOutputStatus extends Packet<FootstepPlanningToolboxOutputStatus> implements Settable<FootstepPlanningToolboxOutputStatus>, EpsilonComparable<FootstepPlanningToolboxOutputStatus>
{
   public static final byte BODY_PATH_PLANNING_RESULT_PLANNING = (byte) 0;
   public static final byte BODY_PATH_PLANNING_RESULT_FOUND_SOLUTION = (byte) 1;
   public static final byte BODY_PATH_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION = (byte) 2;
   public static final byte BODY_PATH_PLANNING_RESULT_NO_PATH_EXISTS = (byte) 3;
   public static final byte BODY_PATH_PLANNING_RESULT_INVALID_GOAL = (byte) 4;
   public static final byte BODY_PATH_PLANNING_RESULT_MAXIMUM_ITERATIONS_REACHED = (byte) 5;
   public static final byte BODY_PATH_PLANNING_RESULT_EXCEPTION = (byte) 6;
   public static final byte BODY_PATH_PLANNING_RESULT_HALTED = (byte) 7;
   public static final byte FOOTSTEP_PLANNING_RESULT_PLANNING = (byte) 0;
   public static final byte FOOTSTEP_PLANNING_RESULT_FOUND_SOLUTION = (byte) 1;
   public static final byte FOOTSTEP_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION = (byte) 2;
   public static final byte FOOTSTEP_PLANNING_RESULT_NO_PATH_EXISTS = (byte) 3;
   public static final byte FOOTSTEP_PLANNING_RESULT_INVALID_GOAL = (byte) 4;
   public static final byte FOOTSTEP_PLANNING_RESULT_MAXIMUM_ITERATIONS_REACHED = (byte) 5;
   public static final byte FOOTSTEP_PLANNING_RESULT_EXCEPTION = (byte) 6;
   public static final byte FOOTSTEP_PLANNING_RESULT_HALTED = (byte) 7;
   public static final int NO_PLAN_ID = -1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * ID of the request this output corresponds to
            */
   public int plan_id_ = -1;
   /**
            * Footstep plan, may be empty depending on the state of the planner, according to footstep_planning_result
            */
   public controller_msgs.msg.dds.FootstepDataListMessage footstep_data_list_;
   /**
            * Body path plan result. Null if no result is available
            */
   public byte body_path_planning_result_ = (byte) 255;
   /**
            * Footstep planner result. Null if no result is available
            */
   public byte footstep_planning_result_ = (byte) 255;
   /**
            * (deprecated) Height map that correspond to the request message. Originally used for debugging networking
            */
   public perception_msgs.msg.dds.HeightMapMessage height_map_message_;
   /**
            * Planned body path. Empty if planner failed
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  body_path_;
   /**
            * Planned body path before smoothing, used for debugging. Empty if planner failed
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  body_path_unsmoothed_;
   /**
            * Goal pose used by the planner. This will be different from the requested goal pose if it's beyond the horizon length.
            */
   public us.ihmc.euclid.geometry.Pose3D goal_pose_;
   /**
            * Object to record various planner timings, helpful for debugging
            */
   public toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage planner_timings_;
   /**
            * Contains planner stack trace if failure is due to an exception
            */
   public java.lang.StringBuilder exception_message_;
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  stacktrace_;

   public FootstepPlanningToolboxOutputStatus()
   {
      footstep_data_list_ = new controller_msgs.msg.dds.FootstepDataListMessage();
      height_map_message_ = new perception_msgs.msg.dds.HeightMapMessage();
      body_path_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (100, new geometry_msgs.msg.dds.PosePubSubType());
      body_path_unsmoothed_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (100, new geometry_msgs.msg.dds.PointPubSubType());
      goal_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      planner_timings_ = new toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage();
      exception_message_ = new java.lang.StringBuilder(255);
      stacktrace_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (20, "type_d");

   }

   public FootstepPlanningToolboxOutputStatus(FootstepPlanningToolboxOutputStatus other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanningToolboxOutputStatus other)
   {
      sequence_id_ = other.sequence_id_;

      plan_id_ = other.plan_id_;

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.staticCopy(other.footstep_data_list_, footstep_data_list_);
      body_path_planning_result_ = other.body_path_planning_result_;

      footstep_planning_result_ = other.footstep_planning_result_;

      perception_msgs.msg.dds.HeightMapMessagePubSubType.staticCopy(other.height_map_message_, height_map_message_);
      body_path_.set(other.body_path_);
      body_path_unsmoothed_.set(other.body_path_unsmoothed_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.goal_pose_, goal_pose_);
      toolbox_msgs.msg.dds.FootstepPlanningTimingsMessagePubSubType.staticCopy(other.planner_timings_, planner_timings_);
      exception_message_.setLength(0);
      exception_message_.append(other.exception_message_);

      stacktrace_.set(other.stacktrace_);
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
            * ID of the request this output corresponds to
            */
   public void setPlanId(int plan_id)
   {
      plan_id_ = plan_id;
   }
   /**
            * ID of the request this output corresponds to
            */
   public int getPlanId()
   {
      return plan_id_;
   }


   /**
            * Footstep plan, may be empty depending on the state of the planner, according to footstep_planning_result
            */
   public controller_msgs.msg.dds.FootstepDataListMessage getFootstepDataList()
   {
      return footstep_data_list_;
   }

   /**
            * Body path plan result. Null if no result is available
            */
   public void setBodyPathPlanningResult(byte body_path_planning_result)
   {
      body_path_planning_result_ = body_path_planning_result;
   }
   /**
            * Body path plan result. Null if no result is available
            */
   public byte getBodyPathPlanningResult()
   {
      return body_path_planning_result_;
   }

   /**
            * Footstep planner result. Null if no result is available
            */
   public void setFootstepPlanningResult(byte footstep_planning_result)
   {
      footstep_planning_result_ = footstep_planning_result;
   }
   /**
            * Footstep planner result. Null if no result is available
            */
   public byte getFootstepPlanningResult()
   {
      return footstep_planning_result_;
   }


   /**
            * (deprecated) Height map that correspond to the request message. Originally used for debugging networking
            */
   public perception_msgs.msg.dds.HeightMapMessage getHeightMapMessage()
   {
      return height_map_message_;
   }


   /**
            * Planned body path. Empty if planner failed
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getBodyPath()
   {
      return body_path_;
   }


   /**
            * Planned body path before smoothing, used for debugging. Empty if planner failed
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getBodyPathUnsmoothed()
   {
      return body_path_unsmoothed_;
   }


   /**
            * Goal pose used by the planner. This will be different from the requested goal pose if it's beyond the horizon length.
            */
   public us.ihmc.euclid.geometry.Pose3D getGoalPose()
   {
      return goal_pose_;
   }


   /**
            * Object to record various planner timings, helpful for debugging
            */
   public toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage getPlannerTimings()
   {
      return planner_timings_;
   }

   /**
            * Contains planner stack trace if failure is due to an exception
            */
   public void setExceptionMessage(java.lang.String exception_message)
   {
      exception_message_.setLength(0);
      exception_message_.append(exception_message);
   }

   /**
            * Contains planner stack trace if failure is due to an exception
            */
   public java.lang.String getExceptionMessageAsString()
   {
      return getExceptionMessage().toString();
   }
   /**
            * Contains planner stack trace if failure is due to an exception
            */
   public java.lang.StringBuilder getExceptionMessage()
   {
      return exception_message_;
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getStacktrace()
   {
      return stacktrace_;
   }


   public static Supplier<FootstepPlanningToolboxOutputStatusPubSubType> getPubSubType()
   {
      return FootstepPlanningToolboxOutputStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanningToolboxOutputStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningToolboxOutputStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.plan_id_, other.plan_id_, epsilon)) return false;

      if (!this.footstep_data_list_.epsilonEquals(other.footstep_data_list_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_path_planning_result_, other.body_path_planning_result_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_planning_result_, other.footstep_planning_result_, epsilon)) return false;

      if (!this.height_map_message_.epsilonEquals(other.height_map_message_, epsilon)) return false;
      if (this.body_path_.size() != other.body_path_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.body_path_.size(); i++)
         {  if (!this.body_path_.get(i).epsilonEquals(other.body_path_.get(i), epsilon)) return false; }
      }

      if (this.body_path_unsmoothed_.size() != other.body_path_unsmoothed_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.body_path_unsmoothed_.size(); i++)
         {  if (!this.body_path_unsmoothed_.get(i).epsilonEquals(other.body_path_unsmoothed_.get(i), epsilon)) return false; }
      }

      if (!this.goal_pose_.epsilonEquals(other.goal_pose_, epsilon)) return false;
      if (!this.planner_timings_.epsilonEquals(other.planner_timings_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.exception_message_, other.exception_message_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.stacktrace_, other.stacktrace_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanningToolboxOutputStatus)) return false;

      FootstepPlanningToolboxOutputStatus otherMyClass = (FootstepPlanningToolboxOutputStatus) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.plan_id_ != otherMyClass.plan_id_) return false;

      if (!this.footstep_data_list_.equals(otherMyClass.footstep_data_list_)) return false;
      if(this.body_path_planning_result_ != otherMyClass.body_path_planning_result_) return false;

      if(this.footstep_planning_result_ != otherMyClass.footstep_planning_result_) return false;

      if (!this.height_map_message_.equals(otherMyClass.height_map_message_)) return false;
      if (!this.body_path_.equals(otherMyClass.body_path_)) return false;
      if (!this.body_path_unsmoothed_.equals(otherMyClass.body_path_unsmoothed_)) return false;
      if (!this.goal_pose_.equals(otherMyClass.goal_pose_)) return false;
      if (!this.planner_timings_.equals(otherMyClass.planner_timings_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.exception_message_, otherMyClass.exception_message_)) return false;

      if (!this.stacktrace_.equals(otherMyClass.stacktrace_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanningToolboxOutputStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("plan_id=");
      builder.append(this.plan_id_);      builder.append(", ");
      builder.append("footstep_data_list=");
      builder.append(this.footstep_data_list_);      builder.append(", ");
      builder.append("body_path_planning_result=");
      builder.append(this.body_path_planning_result_);      builder.append(", ");
      builder.append("footstep_planning_result=");
      builder.append(this.footstep_planning_result_);      builder.append(", ");
      builder.append("height_map_message=");
      builder.append(this.height_map_message_);      builder.append(", ");
      builder.append("body_path=");
      builder.append(this.body_path_);      builder.append(", ");
      builder.append("body_path_unsmoothed=");
      builder.append(this.body_path_unsmoothed_);      builder.append(", ");
      builder.append("goal_pose=");
      builder.append(this.goal_pose_);      builder.append(", ");
      builder.append("planner_timings=");
      builder.append(this.planner_timings_);      builder.append(", ");
      builder.append("exception_message=");
      builder.append(this.exception_message_);      builder.append(", ");
      builder.append("stacktrace=");
      builder.append(this.stacktrace_);
      builder.append("}");
      return builder.toString();
   }
}
