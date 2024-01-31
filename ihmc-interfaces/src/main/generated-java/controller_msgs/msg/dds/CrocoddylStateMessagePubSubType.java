package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CrocoddylStateMessage" defined in "CrocoddylStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CrocoddylStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CrocoddylStateMessage_.idl instead.
*
*/
public class CrocoddylStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.CrocoddylStateMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::CrocoddylStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3f8be21b6db5a48d96052c5f9f24ba19b65949df9d6f0053984947de50a4ecb6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.CrocoddylStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.CrocoddylStateMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getX().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDx().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.CrocoddylStateMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getX().size() <= 100)
      cdr.write_type_e(data.getX());else
          throw new RuntimeException("x field exceeds the maximum length");

      if(data.getDx().size() <= 100)
      cdr.write_type_e(data.getDx());else
          throw new RuntimeException("dx field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.CrocoddylStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getX());	
      cdr.read_type_e(data.getDx());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.CrocoddylStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("x", data.getX());
      ser.write_type_e("dx", data.getDx());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.CrocoddylStateMessage data)
   {
      ser.read_type_e("x", data.getX());
      ser.read_type_e("dx", data.getDx());
   }

   public static void staticCopy(controller_msgs.msg.dds.CrocoddylStateMessage src, controller_msgs.msg.dds.CrocoddylStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.CrocoddylStateMessage createData()
   {
      return new controller_msgs.msg.dds.CrocoddylStateMessage();
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
   
   public void serialize(controller_msgs.msg.dds.CrocoddylStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.CrocoddylStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.CrocoddylStateMessage src, controller_msgs.msg.dds.CrocoddylStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CrocoddylStateMessagePubSubType newInstance()
   {
      return new CrocoddylStateMessagePubSubType();
   }
}
