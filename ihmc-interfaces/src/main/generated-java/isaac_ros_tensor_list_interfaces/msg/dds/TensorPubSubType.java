package isaac_ros_tensor_list_interfaces.msg.dds;

/**
* 
* Topic data type of the struct "Tensor" defined in "Tensor_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Tensor_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Tensor_.idl instead.
*
*/
public class TensorPubSubType implements us.ihmc.pubsub.TopicDataType<isaac_ros_tensor_list_interfaces.msg.dds.Tensor>
{
   public static final java.lang.String name = "isaac_ros_tensor_list_interfaces::msg::dds_::Tensor_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5d7b1378c0a1c07733b22eb43145217883abbcaec7148e3bd0adc356767cc0ce";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(isaac_ros_tensor_list_interfaces.msg.dds.Tensor data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, isaac_ros_tensor_list_interfaces.msg.dds.Tensor data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += isaac_ros_tensor_list_interfaces.msg.dds.TensorShapePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(isaac_ros_tensor_list_interfaces.msg.dds.Tensor data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(isaac_ros_tensor_list_interfaces.msg.dds.Tensor data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

      current_alignment += isaac_ros_tensor_list_interfaces.msg.dds.TensorShapePubSubType.getCdrSerializedSize(data.getShape(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getStrides().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(isaac_ros_tensor_list_interfaces.msg.dds.Tensor data, us.ihmc.idl.CDR cdr)
   {
      if(data.getName().length() <= 255)
      cdr.write_type_d(data.getName());else
          throw new RuntimeException("name field exceeds the maximum length: %d > %d".formatted(data.getName().length(), 255));

      isaac_ros_tensor_list_interfaces.msg.dds.TensorShapePubSubType.write(data.getShape(), cdr);
      cdr.write_type_2(data.getDataType());

      if(data.getStrides().size() <= 100)
      cdr.write_type_e(data.getStrides());else
          throw new RuntimeException("strides field exceeds the maximum length: %d > %d".formatted(data.getStrides().size(), 100));

      if(data.getData().size() <= 100)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length: %d > %d".formatted(data.getData().size(), 100));

   }

   public static void read(isaac_ros_tensor_list_interfaces.msg.dds.Tensor data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getName());	
      isaac_ros_tensor_list_interfaces.msg.dds.TensorShapePubSubType.read(data.getShape(), cdr);	
      data.setDataType(cdr.read_type_2());
      	
      cdr.read_type_e(data.getStrides());	
      cdr.read_type_e(data.getData());	

   }

   @Override
   public final void serialize(isaac_ros_tensor_list_interfaces.msg.dds.Tensor data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("name", data.getName());
      ser.write_type_a("shape", new isaac_ros_tensor_list_interfaces.msg.dds.TensorShapePubSubType(), data.getShape());

      ser.write_type_2("data_type", data.getDataType());
      ser.write_type_e("strides", data.getStrides());
      ser.write_type_e("data", data.getData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, isaac_ros_tensor_list_interfaces.msg.dds.Tensor data)
   {
      ser.read_type_d("name", data.getName());
      ser.read_type_a("shape", new isaac_ros_tensor_list_interfaces.msg.dds.TensorShapePubSubType(), data.getShape());

      data.setDataType(ser.read_type_2("data_type"));
      ser.read_type_e("strides", data.getStrides());
      ser.read_type_e("data", data.getData());
   }

   public static void staticCopy(isaac_ros_tensor_list_interfaces.msg.dds.Tensor src, isaac_ros_tensor_list_interfaces.msg.dds.Tensor dest)
   {
      dest.set(src);
   }

   @Override
   public isaac_ros_tensor_list_interfaces.msg.dds.Tensor createData()
   {
      return new isaac_ros_tensor_list_interfaces.msg.dds.Tensor();
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
   
   public void serialize(isaac_ros_tensor_list_interfaces.msg.dds.Tensor data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(isaac_ros_tensor_list_interfaces.msg.dds.Tensor data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(isaac_ros_tensor_list_interfaces.msg.dds.Tensor src, isaac_ros_tensor_list_interfaces.msg.dds.Tensor dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TensorPubSubType newInstance()
   {
      return new TensorPubSubType();
   }
}
