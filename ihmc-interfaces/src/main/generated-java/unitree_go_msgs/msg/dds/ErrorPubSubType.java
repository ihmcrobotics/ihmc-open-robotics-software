package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Error" defined in "Error_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Error_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Error_.idl instead.
*
*/
public class ErrorPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.Error>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::Error_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "dff9c6f148f46bcf8edab3e1c5f06c6f7139e02721e57d149b92833031e6b4be";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.Error data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.Error data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.Error data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.Error data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.Error data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSource());

      cdr.write_type_4(data.getState());

   }

   public static void read(unitree_go_msgs.msg.dds.Error data, us.ihmc.idl.CDR cdr)
   {
      data.setSource(cdr.read_type_4());
      	
      data.setState(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.Error data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("source", data.getSource());
      ser.write_type_4("state", data.getState());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.Error data)
   {
      data.setSource(ser.read_type_4("source"));
      data.setState(ser.read_type_4("state"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.Error src, unitree_go_msgs.msg.dds.Error dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.Error createData()
   {
      return new unitree_go_msgs.msg.dds.Error();
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
   
   public void serialize(unitree_go_msgs.msg.dds.Error data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.Error data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.Error src, unitree_go_msgs.msg.dds.Error dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ErrorPubSubType newInstance()
   {
      return new ErrorPubSubType();
   }
}
