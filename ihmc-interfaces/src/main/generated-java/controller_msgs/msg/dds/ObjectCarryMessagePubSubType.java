package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ObjectCarryMessage" defined in "ObjectCarryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ObjectCarryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ObjectCarryMessage_.idl instead.
*
*/
public class ObjectCarryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ObjectCarryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ObjectCarryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5c2443de27ea5ce07d24480c449c223c974fcfe0891c3794038dca74a2922341";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ObjectCarryMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ObjectCarryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ObjectCarryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getPickupObject());

   }

   public static void read(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setPickupObject(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("pickup_object", data.getPickupObject());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ObjectCarryMessage data)
   {
      data.setPickupObject(ser.read_type_7("pickup_object"));   }

   public static void staticCopy(controller_msgs.msg.dds.ObjectCarryMessage src, controller_msgs.msg.dds.ObjectCarryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ObjectCarryMessage createData()
   {
      return new controller_msgs.msg.dds.ObjectCarryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ObjectCarryMessage src, controller_msgs.msg.dds.ObjectCarryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ObjectCarryMessagePubSubType newInstance()
   {
      return new ObjectCarryMessagePubSubType();
   }
}
