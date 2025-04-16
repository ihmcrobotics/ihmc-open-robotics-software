package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Res" defined in "Res_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Res_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Res_.idl instead.
*
*/
public class ResPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.Res>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::Res_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "84d92e9c1d14254985401c998fb837a768c36bef4b29d80f913d9b2017e78bba";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.Res data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.Res data) throws java.io.IOException
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
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.Res data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.Res data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getUuid().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getBody().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.Res data, us.ihmc.idl.CDR cdr)
   {
      if(data.getUuid().length() <= 255)
      cdr.write_type_d(data.getUuid());else
          throw new RuntimeException("uuid field exceeds the maximum length: %d > %d".formatted(data.getUuid().length(), 255));

      if(data.getData().size() <= 100)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length: %d > %d".formatted(data.getData().size(), 100));

      if(data.getBody().length() <= 255)
      cdr.write_type_d(data.getBody());else
          throw new RuntimeException("body field exceeds the maximum length: %d > %d".formatted(data.getBody().length(), 255));

   }

   public static void read(unitree_go_msgs.msg.dds.Res data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getUuid());	
      cdr.read_type_e(data.getData());	
      cdr.read_type_d(data.getBody());	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.Res data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("uuid", data.getUuid());
      ser.write_type_e("data", data.getData());
      ser.write_type_d("body", data.getBody());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.Res data)
   {
      ser.read_type_d("uuid", data.getUuid());
      ser.read_type_e("data", data.getData());
      ser.read_type_d("body", data.getBody());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.Res src, unitree_go_msgs.msg.dds.Res dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.Res createData()
   {
      return new unitree_go_msgs.msg.dds.Res();
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
   
   public void serialize(unitree_go_msgs.msg.dds.Res data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.Res data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.Res src, unitree_go_msgs.msg.dds.Res dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ResPubSubType newInstance()
   {
      return new ResPubSubType();
   }
}
