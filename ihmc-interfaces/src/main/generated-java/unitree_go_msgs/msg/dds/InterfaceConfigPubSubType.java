package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "InterfaceConfig" defined in "InterfaceConfig_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from InterfaceConfig_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit InterfaceConfig_.idl instead.
*
*/
public class InterfaceConfigPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.InterfaceConfig>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::InterfaceConfig_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4b86effac7aa98cd60a430a1e90a41eca992b495497f4581884290c7f3d896f2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.InterfaceConfig data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.InterfaceConfig data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((2) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.InterfaceConfig data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.InterfaceConfig data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((2) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.InterfaceConfig data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getMode());

      cdr.write_type_9(data.getValue());

      for(int i0 = 0; i0 < data.getReserve().length; ++i0)
      {
        	cdr.write_type_9(data.getReserve()[i0]);	
      }

   }

   public static void read(unitree_go_msgs.msg.dds.InterfaceConfig data, us.ihmc.idl.CDR cdr)
   {
      data.setMode(cdr.read_type_9());
      	
      data.setValue(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getReserve().length; ++i0)
      {
        	data.getReserve()[i0] = cdr.read_type_9();
        	
      }
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.InterfaceConfig data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("mode", data.getMode());
      ser.write_type_9("value", data.getValue());
      ser.write_type_f("reserve", data.getReserve());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.InterfaceConfig data)
   {
      data.setMode(ser.read_type_9("mode"));
      data.setValue(ser.read_type_9("value"));
      ser.read_type_f("reserve", data.getReserve());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.InterfaceConfig src, unitree_go_msgs.msg.dds.InterfaceConfig dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.InterfaceConfig createData()
   {
      return new unitree_go_msgs.msg.dds.InterfaceConfig();
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
   
   public void serialize(unitree_go_msgs.msg.dds.InterfaceConfig data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.InterfaceConfig data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.InterfaceConfig src, unitree_go_msgs.msg.dds.InterfaceConfig dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public InterfaceConfigPubSubType newInstance()
   {
      return new InterfaceConfigPubSubType();
   }
}
