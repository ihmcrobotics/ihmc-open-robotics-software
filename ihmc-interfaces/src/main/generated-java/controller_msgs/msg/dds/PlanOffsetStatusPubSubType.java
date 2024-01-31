package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PlanOffsetStatus" defined in "PlanOffsetStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PlanOffsetStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PlanOffsetStatus_.idl instead.
*
*/
public class PlanOffsetStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PlanOffsetStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PlanOffsetStatus_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ee104895a03eb443468bc32d576726a0d08b25e73faf5b940b0305280233da4e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PlanOffsetStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PlanOffsetStatus data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanOffsetStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanOffsetStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getOffsetVector(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PlanOffsetStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getOffsetVector(), cdr);
   }

   public static void read(controller_msgs.msg.dds.PlanOffsetStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getOffsetVector(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PlanOffsetStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("offset_vector", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getOffsetVector());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PlanOffsetStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("offset_vector", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getOffsetVector());

   }

   public static void staticCopy(controller_msgs.msg.dds.PlanOffsetStatus src, controller_msgs.msg.dds.PlanOffsetStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PlanOffsetStatus createData()
   {
      return new controller_msgs.msg.dds.PlanOffsetStatus();
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
   
   public void serialize(controller_msgs.msg.dds.PlanOffsetStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PlanOffsetStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PlanOffsetStatus src, controller_msgs.msg.dds.PlanOffsetStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PlanOffsetStatusPubSubType newInstance()
   {
      return new PlanOffsetStatusPubSubType();
   }
}
