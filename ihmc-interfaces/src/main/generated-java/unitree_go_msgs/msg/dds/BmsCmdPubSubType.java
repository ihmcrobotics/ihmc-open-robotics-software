package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BmsCmd" defined in "BmsCmd_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BmsCmd_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BmsCmd_.idl instead.
*
*/
public class BmsCmdPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.BmsCmd>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::BmsCmd_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "36626dcefa027c880295b8044814719e4c0c48a0fc36be340148ba133ef3a03a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.BmsCmd data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.BmsCmd data) throws java.io.IOException
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

      current_alignment += ((3) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.BmsCmd data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.BmsCmd data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((3) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.BmsCmd data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getOff());

      for(int i0 = 0; i0 < data.getReserve().length; ++i0)
      {
        	cdr.write_type_9(data.getReserve()[i0]);	
      }

   }

   public static void read(unitree_go_msgs.msg.dds.BmsCmd data, us.ihmc.idl.CDR cdr)
   {
      data.setOff(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getReserve().length; ++i0)
      {
        	data.getReserve()[i0] = cdr.read_type_9();
        	
      }
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.BmsCmd data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("off", data.getOff());
      ser.write_type_f("reserve", data.getReserve());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.BmsCmd data)
   {
      data.setOff(ser.read_type_9("off"));
      ser.read_type_f("reserve", data.getReserve());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.BmsCmd src, unitree_go_msgs.msg.dds.BmsCmd dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.BmsCmd createData()
   {
      return new unitree_go_msgs.msg.dds.BmsCmd();
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
   
   public void serialize(unitree_go_msgs.msg.dds.BmsCmd data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.BmsCmd data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.BmsCmd src, unitree_go_msgs.msg.dds.BmsCmd dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BmsCmdPubSubType newInstance()
   {
      return new BmsCmdPubSubType();
   }
}
