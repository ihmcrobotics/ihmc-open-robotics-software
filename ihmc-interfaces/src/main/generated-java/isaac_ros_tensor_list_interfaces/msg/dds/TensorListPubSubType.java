package isaac_ros_tensor_list_interfaces.msg.dds;

/**
* 
* Topic data type of the struct "TensorList" defined in "TensorList_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TensorList_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TensorList_.idl instead.
*
*/
public class TensorListPubSubType implements us.ihmc.pubsub.TopicDataType<isaac_ros_tensor_list_interfaces.msg.dds.TensorList>
{
   public static final java.lang.String name = "isaac_ros_tensor_list_interfaces::msg::dds_::TensorList_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8053bb1530c11cab12621ad902a5935453df3174ba55fc457a89284aafdece06";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(isaac_ros_tensor_list_interfaces.msg.dds.TensorList data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, isaac_ros_tensor_list_interfaces.msg.dds.TensorList data) throws java.io.IOException
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

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += isaac_ros_tensor_list_interfaces.msg.dds.TensorPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(isaac_ros_tensor_list_interfaces.msg.dds.TensorList data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(isaac_ros_tensor_list_interfaces.msg.dds.TensorList data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTensors().size(); ++i0)
      {
          current_alignment += isaac_ros_tensor_list_interfaces.msg.dds.TensorPubSubType.getCdrSerializedSize(data.getTensors().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(isaac_ros_tensor_list_interfaces.msg.dds.TensorList data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
      if(data.getTensors().size() <= 100)
      cdr.write_type_e(data.getTensors());else
          throw new RuntimeException("tensors field exceeds the maximum length: %d > %d".formatted(data.getTensors().size(), 100));

   }

   public static void read(isaac_ros_tensor_list_interfaces.msg.dds.TensorList data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);	
      cdr.read_type_e(data.getTensors());	

   }

   @Override
   public final void serialize(isaac_ros_tensor_list_interfaces.msg.dds.TensorList data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_e("tensors", data.getTensors());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, isaac_ros_tensor_list_interfaces.msg.dds.TensorList data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_e("tensors", data.getTensors());
   }

   public static void staticCopy(isaac_ros_tensor_list_interfaces.msg.dds.TensorList src, isaac_ros_tensor_list_interfaces.msg.dds.TensorList dest)
   {
      dest.set(src);
   }

   @Override
   public isaac_ros_tensor_list_interfaces.msg.dds.TensorList createData()
   {
      return new isaac_ros_tensor_list_interfaces.msg.dds.TensorList();
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
   
   public void serialize(isaac_ros_tensor_list_interfaces.msg.dds.TensorList data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(isaac_ros_tensor_list_interfaces.msg.dds.TensorList data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(isaac_ros_tensor_list_interfaces.msg.dds.TensorList src, isaac_ros_tensor_list_interfaces.msg.dds.TensorList dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TensorListPubSubType newInstance()
   {
      return new TensorListPubSubType();
   }
}
