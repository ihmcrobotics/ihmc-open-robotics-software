package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**

 * This message is part of the IHMC whole-body controller API.

 * This message is used to switch the control scheme between different control mode.

 */
public class QuixHighLevelStateMessage extends Packet<QuixHighLevelStateMessage> implements Settable<QuixHighLevelStateMessage>, EpsilonComparable<QuixHighLevelStateMessage>
{

   public static final byte INITIALIZING = (byte) 1;

   public static final byte WAIT_FOR_USER = (byte) 2;

   public static final byte RECOVERABLE_FAULT = (byte) 3;

   public static final byte FATAL_FAULT = (byte) 4;

   public static final byte LIMP = (byte) 5;

   public static final byte WIGGLE = (byte) 6;

   public static final byte HOLD_ALL_READY = (byte) 7;

   public static final byte HOLD_SOME_LOCKED = (byte) 8;

   public static final byte CALL_BEHAVIOR = (byte) 9;

   /**

    * Unique ID used to identify this message, should preferably be consecutively increasing.

    */
   public long sequence_id_;

   /**

    * Specifies the which state the controller should transition into.

    */
   public byte high_level_state_name_ = (byte) 255;

   public QuixHighLevelStateMessage()
   {



   }

   public QuixHighLevelStateMessage(QuixHighLevelStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixHighLevelStateMessage other)
   {

      sequence_id_ = other.sequence_id_;


      high_level_state_name_ = other.high_level_state_name_;

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

    * Specifies the which state the controller should transition into.

    */
   public void setHighLevelStateName(byte high_level_state_name)
   {
      high_level_state_name_ = high_level_state_name;
   }
   /**

    * Specifies the which state the controller should transition into.

    */
   public byte getHighLevelStateName()
   {
      return high_level_state_name_;
   }


   public static Supplier<QuixHighLevelStateMessagePubSubType> getPubSubType()
   {
      return QuixHighLevelStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixHighLevelStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixHighLevelStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.high_level_state_name_, other.high_level_state_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixHighLevelStateMessage)) return false;

      QuixHighLevelStateMessage otherMyClass = (QuixHighLevelStateMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.high_level_state_name_ != otherMyClass.high_level_state_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixHighLevelStateMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("high_level_state_name=");
      builder.append(this.high_level_state_name_);
      builder.append("}");
      return builder.toString();
   }
}
