package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WeightMatrix3DMessage" defined in "WeightMatrix3DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WeightMatrix3DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WeightMatrix3DMessage_.idl instead.
*
*/
public class WeightMatrix3DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.WeightMatrix3DMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::WeightMatrix3DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1c46d3548addd4a8b7eb27e4e41b837b6d3d643683f5eff8e132583ea6d8d1fd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getWeightFrameId());

      cdr.write_type_6(data.getXWeight());

      cdr.write_type_6(data.getYWeight());

      cdr.write_type_6(data.getZWeight());

   }

   public static void read(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setWeightFrameId(cdr.read_type_11());
      	
      data.setXWeight(cdr.read_type_6());
      	
      data.setYWeight(cdr.read_type_6());
      	
      data.setZWeight(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("weight_frame_id", data.getWeightFrameId());
      ser.write_type_6("x_weight", data.getXWeight());
      ser.write_type_6("y_weight", data.getYWeight());
      ser.write_type_6("z_weight", data.getZWeight());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setWeightFrameId(ser.read_type_11("weight_frame_id"));
      data.setXWeight(ser.read_type_6("x_weight"));
      data.setYWeight(ser.read_type_6("y_weight"));
      data.setZWeight(ser.read_type_6("z_weight"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage src, ihmc_common_msgs.msg.dds.WeightMatrix3DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.WeightMatrix3DMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.WeightMatrix3DMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.WeightMatrix3DMessage src, ihmc_common_msgs.msg.dds.WeightMatrix3DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WeightMatrix3DMessagePubSubType newInstance()
   {
      return new WeightMatrix3DMessagePubSubType();
   }
}
