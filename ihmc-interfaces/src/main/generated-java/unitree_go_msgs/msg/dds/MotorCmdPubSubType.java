package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MotorCmd" defined in "MotorCmd_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MotorCmd_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MotorCmd_.idl instead.
*
*/
public class MotorCmdPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.MotorCmd>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::MotorCmd_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "06da4d76cb625a412b832260014d5336c302692c232de23389a6230bc64f29e1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.MotorCmd data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.MotorCmd data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.MotorCmd data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.MotorCmd data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.MotorCmd data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getMode());

      cdr.write_type_5(data.getQ());

      cdr.write_type_5(data.getDq());

      cdr.write_type_5(data.getTau());

      cdr.write_type_5(data.getKp());

      cdr.write_type_5(data.getKd());

   }

   public static void read(unitree_go_msgs.msg.dds.MotorCmd data, us.ihmc.idl.CDR cdr)
   {
      data.setMode(cdr.read_type_9());
      	
      data.setQ(cdr.read_type_5());
      	
      data.setDq(cdr.read_type_5());
      	
      data.setTau(cdr.read_type_5());
      	
      data.setKp(cdr.read_type_5());
      	
      data.setKd(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.MotorCmd data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("mode", data.getMode());
      ser.write_type_5("q", data.getQ());
      ser.write_type_5("dq", data.getDq());
      ser.write_type_5("tau", data.getTau());
      ser.write_type_5("kp", data.getKp());
      ser.write_type_5("kd", data.getKd());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.MotorCmd data)
   {
      data.setMode(ser.read_type_9("mode"));
      data.setQ(ser.read_type_5("q"));
      data.setDq(ser.read_type_5("dq"));
      data.setTau(ser.read_type_5("tau"));
      data.setKp(ser.read_type_5("kp"));
      data.setKd(ser.read_type_5("kd"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.MotorCmd src, unitree_go_msgs.msg.dds.MotorCmd dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.MotorCmd createData()
   {
      return new unitree_go_msgs.msg.dds.MotorCmd();
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
   
   public void serialize(unitree_go_msgs.msg.dds.MotorCmd data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.MotorCmd data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.MotorCmd src, unitree_go_msgs.msg.dds.MotorCmd dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MotorCmdPubSubType newInstance()
   {
      return new MotorCmdPubSubType();
   }
}
