package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PIDGainsMessage" defined in "PIDGainsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PIDGainsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PIDGainsMessage_.idl instead.
*
*/
public class PIDGainsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PIDGainsMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PIDGainsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8970630ccb78c71075c429d35c53b60ff74f267b348b6af6d772829a805c2dea";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PIDGainsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PIDGainsMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PIDGainsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PIDGainsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PIDGainsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getKp());

      cdr.write_type_6(data.getKd());

      cdr.write_type_6(data.getKi());

      cdr.write_type_6(data.getZeta());

   }

   public static void read(controller_msgs.msg.dds.PIDGainsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setKp(cdr.read_type_6());
      	
      data.setKd(cdr.read_type_6());
      	
      data.setKi(cdr.read_type_6());
      	
      data.setZeta(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PIDGainsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("kp", data.getKp());
      ser.write_type_6("kd", data.getKd());
      ser.write_type_6("ki", data.getKi());
      ser.write_type_6("zeta", data.getZeta());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PIDGainsMessage data)
   {
      data.setKp(ser.read_type_6("kp"));
      data.setKd(ser.read_type_6("kd"));
      data.setKi(ser.read_type_6("ki"));
      data.setZeta(ser.read_type_6("zeta"));
   }

   public static void staticCopy(controller_msgs.msg.dds.PIDGainsMessage src, controller_msgs.msg.dds.PIDGainsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PIDGainsMessage createData()
   {
      return new controller_msgs.msg.dds.PIDGainsMessage();
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
   
   public void serialize(controller_msgs.msg.dds.PIDGainsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PIDGainsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PIDGainsMessage src, controller_msgs.msg.dds.PIDGainsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PIDGainsMessagePubSubType newInstance()
   {
      return new PIDGainsMessagePubSubType();
   }
}
