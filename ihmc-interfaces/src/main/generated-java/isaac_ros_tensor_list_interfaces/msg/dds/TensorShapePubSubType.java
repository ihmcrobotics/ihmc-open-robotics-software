package isaac_ros_tensor_list_interfaces.msg.dds;

/**
* 
* Topic data type of the struct "TensorShape" defined in "TensorShape_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TensorShape_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TensorShape_.idl instead.
*
*/
public class TensorShapePubSubType implements us.ihmc.pubsub.TopicDataType<isaac_ros_tensor_list_interfaces.msg.dds.TensorShape>
{
   public static final java.lang.String name = "isaac_ros_tensor_list_interfaces::msg::dds_::TensorShape_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "14f1eacabd9695d2a3a83d1b284f4a94380d2579cf2d0ed6865b57b08e2b56e7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDims().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRank());

      if(data.getDims().size() <= 100)
      cdr.write_type_e(data.getDims());else
          throw new RuntimeException("dims field exceeds the maximum length: %d > %d".formatted(data.getDims().size(), 100));

   }

   public static void read(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data, us.ihmc.idl.CDR cdr)
   {
      data.setRank(cdr.read_type_9());
      	
      cdr.read_type_e(data.getDims());	

   }

   @Override
   public final void serialize(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("rank", data.getRank());
      ser.write_type_e("dims", data.getDims());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data)
   {
      data.setRank(ser.read_type_9("rank"));
      ser.read_type_e("dims", data.getDims());
   }

   public static void staticCopy(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape src, isaac_ros_tensor_list_interfaces.msg.dds.TensorShape dest)
   {
      dest.set(src);
   }

   @Override
   public isaac_ros_tensor_list_interfaces.msg.dds.TensorShape createData()
   {
      return new isaac_ros_tensor_list_interfaces.msg.dds.TensorShape();
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
   
   public void serialize(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(isaac_ros_tensor_list_interfaces.msg.dds.TensorShape src, isaac_ros_tensor_list_interfaces.msg.dds.TensorShape dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TensorShapePubSubType newInstance()
   {
      return new TensorShapePubSubType();
   }
}
