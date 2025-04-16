package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TimeSpec" defined in "TimeSpec_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TimeSpec_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TimeSpec_.idl instead.
*
*/
public class TimeSpecPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.TimeSpec>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::TimeSpec_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d13fc56ac7d402014a0290b3d2138aa1d7c9a0b4795d1617e6f2cdb181bfaf73";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.TimeSpec data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.TimeSpec data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.TimeSpec data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.TimeSpec data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.TimeSpec data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getSec());

      cdr.write_type_4(data.getNanosec());

   }

   public static void read(unitree_go_msgs.msg.dds.TimeSpec data, us.ihmc.idl.CDR cdr)
   {
      data.setSec(cdr.read_type_2());
      	
      data.setNanosec(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.TimeSpec data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("sec", data.getSec());
      ser.write_type_4("nanosec", data.getNanosec());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.TimeSpec data)
   {
      data.setSec(ser.read_type_2("sec"));
      data.setNanosec(ser.read_type_4("nanosec"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.TimeSpec src, unitree_go_msgs.msg.dds.TimeSpec dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.TimeSpec createData()
   {
      return new unitree_go_msgs.msg.dds.TimeSpec();
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
   
   public void serialize(unitree_go_msgs.msg.dds.TimeSpec data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.TimeSpec data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.TimeSpec src, unitree_go_msgs.msg.dds.TimeSpec dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TimeSpecPubSubType newInstance()
   {
      return new TimeSpecPubSubType();
   }
}
