package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       * The planner recomputes swing waypoints for the last planned path if it receives this message.
       * If no path was previously planned, it's ignored.
       */
public class SwingPlanningRequestPacket extends Packet<SwingPlanningRequestPacket> implements Settable<SwingPlanningRequestPacket>, EpsilonComparable<SwingPlanningRequestPacket>
{
   public static final byte SWING_PLANNER_TYPE_NONE = (byte) 0;
   public static final byte SWING_PLANNER_TYPE_POSITION = (byte) 1;
   public static final byte SWING_PLANNER_TYPE_PROPORTION = (byte) 2;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies swing planner to use. See the above enumeration
            */
   public byte requested_swing_planner_;
   /**
            * Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
            * For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
            */
   public boolean generate_log_;

   public SwingPlanningRequestPacket()
   {
   }

   public SwingPlanningRequestPacket(SwingPlanningRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(SwingPlanningRequestPacket other)
   {
      sequence_id_ = other.sequence_id_;

      requested_swing_planner_ = other.requested_swing_planner_;

      generate_log_ = other.generate_log_;

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
            * Specifies swing planner to use. See the above enumeration
            */
   public void setRequestedSwingPlanner(byte requested_swing_planner)
   {
      requested_swing_planner_ = requested_swing_planner;
   }
   /**
            * Specifies swing planner to use. See the above enumeration
            */
   public byte getRequestedSwingPlanner()
   {
      return requested_swing_planner_;
   }

   /**
            * Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
            * For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
            */
   public void setGenerateLog(boolean generate_log)
   {
      generate_log_ = generate_log;
   }
   /**
            * Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
            * For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
            */
   public boolean getGenerateLog()
   {
      return generate_log_;
   }


   public static Supplier<SwingPlanningRequestPacketPubSubType> getPubSubType()
   {
      return SwingPlanningRequestPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SwingPlanningRequestPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SwingPlanningRequestPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_swing_planner_, other.requested_swing_planner_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.generate_log_, other.generate_log_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SwingPlanningRequestPacket)) return false;

      SwingPlanningRequestPacket otherMyClass = (SwingPlanningRequestPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.requested_swing_planner_ != otherMyClass.requested_swing_planner_) return false;

      if(this.generate_log_ != otherMyClass.generate_log_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SwingPlanningRequestPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("requested_swing_planner=");
      builder.append(this.requested_swing_planner_);      builder.append(", ");
      builder.append("generate_log=");
      builder.append(this.generate_log_);
      builder.append("}");
      return builder.toString();
   }
}
