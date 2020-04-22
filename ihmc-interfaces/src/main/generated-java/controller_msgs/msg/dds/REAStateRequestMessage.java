package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC robot environment awareness module (REA).
       * It is destined to host user requests for changing the internal state of REA.
       */
public class REAStateRequestMessage extends Packet<REAStateRequestMessage> implements Settable<REAStateRequestMessage>, EpsilonComparable<REAStateRequestMessage>
{

   /**
            * When true, REA will temporarily stop collecting pointcloud data until another message requesting it to resume is sent.
            * In the case REA was already paused, nothing changes.
            * In the case both pause and resume are requested, the pause request is ignored.
            */
   public boolean request_pause_;

   /**
            * When true, REA will resume collecting pointcloud data.
            * In the case REA was already collecting data, nothing changes.
            * In the case both pause and resume are requested, the pause request is ignored.
            */
   public boolean request_resume_;

   /**
            * When true, REA will clear its internal data collected until now.
            * After a clear, REA will start building planar regions from scratch.
            * This is useful in the case of noisy data, of if the sensor pose estimation has drifted for instance.
            */
   public boolean request_clear_;

   public REAStateRequestMessage()
   {




   }

   public REAStateRequestMessage(REAStateRequestMessage other)
   {
      this();
      set(other);
   }

   public void set(REAStateRequestMessage other)
   {

      request_pause_ = other.request_pause_;


      request_resume_ = other.request_resume_;


      request_clear_ = other.request_clear_;

   }


   /**
            * When true, REA will temporarily stop collecting pointcloud data until another message requesting it to resume is sent.
            * In the case REA was already paused, nothing changes.
            * In the case both pause and resume are requested, the pause request is ignored.
            */
   public void setRequestPause(boolean request_pause)
   {
      request_pause_ = request_pause;
   }
   /**
            * When true, REA will temporarily stop collecting pointcloud data until another message requesting it to resume is sent.
            * In the case REA was already paused, nothing changes.
            * In the case both pause and resume are requested, the pause request is ignored.
            */
   public boolean getRequestPause()
   {
      return request_pause_;
   }


   /**
            * When true, REA will resume collecting pointcloud data.
            * In the case REA was already collecting data, nothing changes.
            * In the case both pause and resume are requested, the pause request is ignored.
            */
   public void setRequestResume(boolean request_resume)
   {
      request_resume_ = request_resume;
   }
   /**
            * When true, REA will resume collecting pointcloud data.
            * In the case REA was already collecting data, nothing changes.
            * In the case both pause and resume are requested, the pause request is ignored.
            */
   public boolean getRequestResume()
   {
      return request_resume_;
   }


   /**
            * When true, REA will clear its internal data collected until now.
            * After a clear, REA will start building planar regions from scratch.
            * This is useful in the case of noisy data, of if the sensor pose estimation has drifted for instance.
            */
   public void setRequestClear(boolean request_clear)
   {
      request_clear_ = request_clear;
   }
   /**
            * When true, REA will clear its internal data collected until now.
            * After a clear, REA will start building planar regions from scratch.
            * This is useful in the case of noisy data, of if the sensor pose estimation has drifted for instance.
            */
   public boolean getRequestClear()
   {
      return request_clear_;
   }


   public static Supplier<REAStateRequestMessagePubSubType> getPubSubType()
   {
      return REAStateRequestMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return REAStateRequestMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(REAStateRequestMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_pause_, other.request_pause_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_resume_, other.request_resume_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_clear_, other.request_clear_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof REAStateRequestMessage)) return false;

      REAStateRequestMessage otherMyClass = (REAStateRequestMessage) other;


      if(this.request_pause_ != otherMyClass.request_pause_) return false;


      if(this.request_resume_ != otherMyClass.request_resume_) return false;


      if(this.request_clear_ != otherMyClass.request_clear_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("REAStateRequestMessage {");

      builder.append("request_pause=");
      builder.append(this.request_pause_);      builder.append(", ");

      builder.append("request_resume=");
      builder.append(this.request_resume_);      builder.append(", ");

      builder.append("request_clear=");
      builder.append(this.request_clear_);
      builder.append("}");
      return builder.toString();
   }
}
