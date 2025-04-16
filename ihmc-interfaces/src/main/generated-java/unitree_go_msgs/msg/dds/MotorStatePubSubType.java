package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MotorState" defined in "MotorState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MotorState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MotorState_.idl instead.
*
*/
public class MotorStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.MotorState>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::MotorState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "797a51924788e43f631812a89ec302d81b86d9e7d45beef9deea7a03d8045725";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.MotorState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.MotorState data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.MotorState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.MotorState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.MotorState data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getMode());

      cdr.write_type_5(data.getQ());

      cdr.write_type_5(data.getDq());

      cdr.write_type_5(data.getDdq());

      cdr.write_type_5(data.getTauEst());

      cdr.write_type_5(data.getQRaw());

      cdr.write_type_5(data.getDqRaw());

      cdr.write_type_5(data.getDdqRaw());

      cdr.write_type_9(data.getTemperature());

      cdr.write_type_4(data.getLost());

   }

   public static void read(unitree_go_msgs.msg.dds.MotorState data, us.ihmc.idl.CDR cdr)
   {
      data.setMode(cdr.read_type_9());
      	
      data.setQ(cdr.read_type_5());
      	
      data.setDq(cdr.read_type_5());
      	
      data.setDdq(cdr.read_type_5());
      	
      data.setTauEst(cdr.read_type_5());
      	
      data.setQRaw(cdr.read_type_5());
      	
      data.setDqRaw(cdr.read_type_5());
      	
      data.setDdqRaw(cdr.read_type_5());
      	
      data.setTemperature(cdr.read_type_9());
      	
      data.setLost(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.MotorState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("mode", data.getMode());
      ser.write_type_5("q", data.getQ());
      ser.write_type_5("dq", data.getDq());
      ser.write_type_5("ddq", data.getDdq());
      ser.write_type_5("tau_est", data.getTauEst());
      ser.write_type_5("q_raw", data.getQRaw());
      ser.write_type_5("dq_raw", data.getDqRaw());
      ser.write_type_5("ddq_raw", data.getDdqRaw());
      ser.write_type_9("temperature", data.getTemperature());
      ser.write_type_4("lost", data.getLost());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.MotorState data)
   {
      data.setMode(ser.read_type_9("mode"));
      data.setQ(ser.read_type_5("q"));
      data.setDq(ser.read_type_5("dq"));
      data.setDdq(ser.read_type_5("ddq"));
      data.setTauEst(ser.read_type_5("tau_est"));
      data.setQRaw(ser.read_type_5("q_raw"));
      data.setDqRaw(ser.read_type_5("dq_raw"));
      data.setDdqRaw(ser.read_type_5("ddq_raw"));
      data.setTemperature(ser.read_type_9("temperature"));
      data.setLost(ser.read_type_4("lost"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.MotorState src, unitree_go_msgs.msg.dds.MotorState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.MotorState createData()
   {
      return new unitree_go_msgs.msg.dds.MotorState();
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
   
   public void serialize(unitree_go_msgs.msg.dds.MotorState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.MotorState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.MotorState src, unitree_go_msgs.msg.dds.MotorState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MotorStatePubSubType newInstance()
   {
      return new MotorStatePubSubType();
   }
}
