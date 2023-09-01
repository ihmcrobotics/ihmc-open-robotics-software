package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "IntrinsicParametersMessage" defined in "IntrinsicParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from IntrinsicParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit IntrinsicParametersMessage_.idl instead.
*
*/
public class IntrinsicParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.IntrinsicParametersMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::IntrinsicParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0a1a602ede9c3af8da2d4e5dc23fd4a3bc3d1b5f68a085b896a9ddb04e53c45e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.IntrinsicParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.IntrinsicParametersMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.IntrinsicParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.IntrinsicParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getRadial().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.IntrinsicParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getWidth());

      cdr.write_type_2(data.getHeight());

      cdr.write_type_6(data.getFx());

      cdr.write_type_6(data.getFy());

      cdr.write_type_6(data.getSkew());

      cdr.write_type_6(data.getCx());

      cdr.write_type_6(data.getCy());

      if(data.getRadial().size() <= 100)
      cdr.write_type_e(data.getRadial());else
          throw new RuntimeException("radial field exceeds the maximum length");

      cdr.write_type_6(data.getT1());

      cdr.write_type_6(data.getT2());

   }

   public static void read(perception_msgs.msg.dds.IntrinsicParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setWidth(cdr.read_type_2());
      	
      data.setHeight(cdr.read_type_2());
      	
      data.setFx(cdr.read_type_6());
      	
      data.setFy(cdr.read_type_6());
      	
      data.setSkew(cdr.read_type_6());
      	
      data.setCx(cdr.read_type_6());
      	
      data.setCy(cdr.read_type_6());
      	
      cdr.read_type_e(data.getRadial());	
      data.setT1(cdr.read_type_6());
      	
      data.setT2(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.IntrinsicParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("width", data.getWidth());
      ser.write_type_2("height", data.getHeight());
      ser.write_type_6("fx", data.getFx());
      ser.write_type_6("fy", data.getFy());
      ser.write_type_6("skew", data.getSkew());
      ser.write_type_6("cx", data.getCx());
      ser.write_type_6("cy", data.getCy());
      ser.write_type_e("radial", data.getRadial());
      ser.write_type_6("t1", data.getT1());
      ser.write_type_6("t2", data.getT2());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.IntrinsicParametersMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setWidth(ser.read_type_2("width"));
      data.setHeight(ser.read_type_2("height"));
      data.setFx(ser.read_type_6("fx"));
      data.setFy(ser.read_type_6("fy"));
      data.setSkew(ser.read_type_6("skew"));
      data.setCx(ser.read_type_6("cx"));
      data.setCy(ser.read_type_6("cy"));
      ser.read_type_e("radial", data.getRadial());
      data.setT1(ser.read_type_6("t1"));
      data.setT2(ser.read_type_6("t2"));
   }

   public static void staticCopy(perception_msgs.msg.dds.IntrinsicParametersMessage src, perception_msgs.msg.dds.IntrinsicParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.IntrinsicParametersMessage createData()
   {
      return new perception_msgs.msg.dds.IntrinsicParametersMessage();
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
   
   public void serialize(perception_msgs.msg.dds.IntrinsicParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.IntrinsicParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.IntrinsicParametersMessage src, perception_msgs.msg.dds.IntrinsicParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public IntrinsicParametersMessagePubSubType newInstance()
   {
      return new IntrinsicParametersMessagePubSubType();
   }
}
