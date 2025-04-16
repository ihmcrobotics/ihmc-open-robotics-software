package unitree_h_one_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MotorState" defined in "MotorState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MotorState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MotorState_.idl instead.
*
*/
public class MotorStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_h_one_msgs.msg.dds.MotorState>
{
   public static final java.lang.String name = "unitree_h_one_msgs::msg::dds_::MotorState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c81f7ff52a44a9bf4c75621e4880e13b356f85eefc7226e1733a0baec5a5e691";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_h_one_msgs.msg.dds.MotorState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_h_one_msgs.msg.dds.MotorState data) throws java.io.IOException
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

      current_alignment += ((2) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.MotorState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.MotorState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((2) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_h_one_msgs.msg.dds.MotorState data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getMode());

      cdr.write_type_5(data.getQ());

      cdr.write_type_5(data.getDq());

      cdr.write_type_5(data.getDdq());

      cdr.write_type_5(data.getTauEst());

      for(int i0 = 0; i0 < data.getTemperature().length; ++i0)
      {
        	cdr.write_type_1(data.getTemperature()[i0]);	
      }

      cdr.write_type_5(data.getVol());

      cdr.write_type_4(data.getMotorstate());

   }

   public static void read(unitree_h_one_msgs.msg.dds.MotorState data, us.ihmc.idl.CDR cdr)
   {
      data.setMode(cdr.read_type_9());
      	
      data.setQ(cdr.read_type_5());
      	
      data.setDq(cdr.read_type_5());
      	
      data.setDdq(cdr.read_type_5());
      	
      data.setTauEst(cdr.read_type_5());
      	
      for(int i0 = 0; i0 < data.getTemperature().length; ++i0)
      {
        	data.getTemperature()[i0] = cdr.read_type_1();
        	
      }
      	
      data.setVol(cdr.read_type_5());
      	
      data.setMotorstate(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(unitree_h_one_msgs.msg.dds.MotorState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("mode", data.getMode());
      ser.write_type_5("q", data.getQ());
      ser.write_type_5("dq", data.getDq());
      ser.write_type_5("ddq", data.getDdq());
      ser.write_type_5("tau_est", data.getTauEst());
      ser.write_type_f("temperature", data.getTemperature());
      ser.write_type_5("vol", data.getVol());
      ser.write_type_4("motorstate", data.getMotorstate());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_h_one_msgs.msg.dds.MotorState data)
   {
      data.setMode(ser.read_type_9("mode"));
      data.setQ(ser.read_type_5("q"));
      data.setDq(ser.read_type_5("dq"));
      data.setDdq(ser.read_type_5("ddq"));
      data.setTauEst(ser.read_type_5("tau_est"));
      ser.read_type_f("temperature", data.getTemperature());
      data.setVol(ser.read_type_5("vol"));
      data.setMotorstate(ser.read_type_4("motorstate"));
   }

   public static void staticCopy(unitree_h_one_msgs.msg.dds.MotorState src, unitree_h_one_msgs.msg.dds.MotorState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_h_one_msgs.msg.dds.MotorState createData()
   {
      return new unitree_h_one_msgs.msg.dds.MotorState();
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
   
   public void serialize(unitree_h_one_msgs.msg.dds.MotorState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_h_one_msgs.msg.dds.MotorState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_h_one_msgs.msg.dds.MotorState src, unitree_h_one_msgs.msg.dds.MotorState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MotorStatePubSubType newInstance()
   {
      return new MotorStatePubSubType();
   }
}
