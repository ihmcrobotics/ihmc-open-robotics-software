package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Float32MultiArrayHack" defined in "Float32MultiArrayHack_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Float32MultiArrayHack_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Float32MultiArrayHack_.idl instead.
*
*/
public class Float32MultiArrayHackPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.Float32MultiArrayHack>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::Float32MultiArrayHack_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "635e246c1837ec880e28f90bb4b7a445d7ebd6ba7808071139287dd15ccce0ac";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.Float32MultiArrayHack data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.Float32MultiArrayHack data) throws java.io.IOException
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

      current_alignment += std_msgs.msg.dds.MultiArrayLayoutPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (1024 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.Float32MultiArrayHack data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.Float32MultiArrayHack data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.MultiArrayLayoutPubSubType.getCdrSerializedSize(data.getLayout(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.Float32MultiArrayHack data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.MultiArrayLayoutPubSubType.write(data.getLayout(), cdr);
      if(data.getData().size() <= 1024)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length: %d > %d".formatted(data.getData().size(), 1024));

   }

   public static void read(perception_msgs.msg.dds.Float32MultiArrayHack data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.MultiArrayLayoutPubSubType.read(data.getLayout(), cdr);	
      cdr.read_type_e(data.getData());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.Float32MultiArrayHack data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("layout", new std_msgs.msg.dds.MultiArrayLayoutPubSubType(), data.getLayout());

      ser.write_type_e("data", data.getData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.Float32MultiArrayHack data)
   {
      ser.read_type_a("layout", new std_msgs.msg.dds.MultiArrayLayoutPubSubType(), data.getLayout());

      ser.read_type_e("data", data.getData());
   }

   public static void staticCopy(perception_msgs.msg.dds.Float32MultiArrayHack src, perception_msgs.msg.dds.Float32MultiArrayHack dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.Float32MultiArrayHack createData()
   {
      return new perception_msgs.msg.dds.Float32MultiArrayHack();
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
   
   public void serialize(perception_msgs.msg.dds.Float32MultiArrayHack data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.Float32MultiArrayHack data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.Float32MultiArrayHack src, perception_msgs.msg.dds.Float32MultiArrayHack dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Float32MultiArrayHackPubSubType newInstance()
   {
      return new Float32MultiArrayHackPubSubType();
   }
}
