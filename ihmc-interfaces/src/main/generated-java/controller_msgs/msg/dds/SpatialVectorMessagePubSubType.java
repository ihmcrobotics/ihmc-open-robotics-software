package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SpatialVectorMessage" defined in "SpatialVectorMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SpatialVectorMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SpatialVectorMessage_.idl instead.
*
*/
public class SpatialVectorMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SpatialVectorMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SpatialVectorMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "63c5f509f74604a10b703187cfc8be5dda793a6a96333b9751e2c0079b530257";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SpatialVectorMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SpatialVectorMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SpatialVectorMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SpatialVectorMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getAngularPart(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getLinearPart(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SpatialVectorMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getAngularPart(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getLinearPart(), cdr);
   }

   public static void read(controller_msgs.msg.dds.SpatialVectorMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getAngularPart(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getLinearPart(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SpatialVectorMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("angular_part", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularPart());

      ser.write_type_a("linear_part", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinearPart());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SpatialVectorMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("angular_part", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularPart());

      ser.read_type_a("linear_part", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinearPart());

   }

   public static void staticCopy(controller_msgs.msg.dds.SpatialVectorMessage src, controller_msgs.msg.dds.SpatialVectorMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SpatialVectorMessage createData()
   {
      return new controller_msgs.msg.dds.SpatialVectorMessage();
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
   
   public void serialize(controller_msgs.msg.dds.SpatialVectorMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SpatialVectorMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SpatialVectorMessage src, controller_msgs.msg.dds.SpatialVectorMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SpatialVectorMessagePubSubType newInstance()
   {
      return new SpatialVectorMessagePubSubType();
   }
}
