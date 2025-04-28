package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "GuidMessage" defined in "GuidMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from GuidMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit GuidMessage_.idl instead.
*
*/
public class GuidMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.GuidMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::GuidMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5f61ddf5c243dbf8995931ed6448c975b957a0835684ac6064ec5e810f93607d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.GuidMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.GuidMessage data) throws java.io.IOException
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

      current_alignment += ((12) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((4) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.GuidMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.GuidMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ((12) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += ((4) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.GuidMessage data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getPrefix().length; ++i0)
      {
        	cdr.write_type_9(data.getPrefix()[i0]);	
      }

      for(int i0 = 0; i0 < data.getEntity().length; ++i0)
      {
        	cdr.write_type_9(data.getEntity()[i0]);	
      }

   }

   public static void read(ihmc_common_msgs.msg.dds.GuidMessage data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getPrefix().length; ++i0)
      {
        	data.getPrefix()[i0] = cdr.read_type_9();
        	
      }
      	
      for(int i0 = 0; i0 < data.getEntity().length; ++i0)
      {
        	data.getEntity()[i0] = cdr.read_type_9();
        	
      }
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.GuidMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_f("prefix", data.getPrefix());
      ser.write_type_f("entity", data.getEntity());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.GuidMessage data)
   {
      ser.read_type_f("prefix", data.getPrefix());
      ser.read_type_f("entity", data.getEntity());
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.GuidMessage src, ihmc_common_msgs.msg.dds.GuidMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.GuidMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.GuidMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.GuidMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.GuidMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.GuidMessage src, ihmc_common_msgs.msg.dds.GuidMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public GuidMessagePubSubType newInstance()
   {
      return new GuidMessagePubSubType();
   }
}
