package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CrocoddylTimeIntervalMessage" defined in "CrocoddylTimeIntervalMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CrocoddylTimeIntervalMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CrocoddylTimeIntervalMessage_.idl instead.
*
*/
public class CrocoddylTimeIntervalMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.CrocoddylTimeIntervalMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::CrocoddylTimeIntervalMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "14f6e7b87eb4c17d9ca4ce42e1c8c5c27a7174df66bfc2c02007850bc1266b01";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getTime());

      cdr.write_type_6(data.getDuration());

   }

   public static void read(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setTime(cdr.read_type_6());
      	
      data.setDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("time", data.getTime());
      ser.write_type_6("duration", data.getDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data)
   {
      data.setTime(ser.read_type_6("time"));
      data.setDuration(ser.read_type_6("duration"));
   }

   public static void staticCopy(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage src, controller_msgs.msg.dds.CrocoddylTimeIntervalMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.CrocoddylTimeIntervalMessage createData()
   {
      return new controller_msgs.msg.dds.CrocoddylTimeIntervalMessage();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.CrocoddylTimeIntervalMessage src, controller_msgs.msg.dds.CrocoddylTimeIntervalMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CrocoddylTimeIntervalMessagePubSubType newInstance()
   {
      return new CrocoddylTimeIntervalMessagePubSubType();
   }
}
