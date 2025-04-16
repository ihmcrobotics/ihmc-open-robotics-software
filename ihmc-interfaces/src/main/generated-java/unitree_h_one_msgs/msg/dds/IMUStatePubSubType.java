package unitree_h_one_msgs.msg.dds;

/**
* 
* Topic data type of the struct "IMUState" defined in "IMUState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from IMUState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit IMUState_.idl instead.
*
*/
public class IMUStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_h_one_msgs.msg.dds.IMUState>
{
   public static final java.lang.String name = "unitree_h_one_msgs::msg::dds_::IMUState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1a23375e5f9997e08c6c7ee608078ddd8eec12225fc03e6975ba6f798079b3de";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_h_one_msgs.msg.dds.IMUState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_h_one_msgs.msg.dds.IMUState data) throws java.io.IOException
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

      current_alignment += ((4) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.IMUState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.IMUState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ((4) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_h_one_msgs.msg.dds.IMUState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getQuaternion().length; ++i0)
      {
        	cdr.write_type_5(data.getQuaternion()[i0]);	
      }

      for(int i0 = 0; i0 < data.getGyroscope().length; ++i0)
      {
        	cdr.write_type_5(data.getGyroscope()[i0]);	
      }

      for(int i0 = 0; i0 < data.getAccelerometer().length; ++i0)
      {
        	cdr.write_type_5(data.getAccelerometer()[i0]);	
      }

      for(int i0 = 0; i0 < data.getRpy().length; ++i0)
      {
        	cdr.write_type_5(data.getRpy()[i0]);	
      }

      cdr.write_type_1(data.getTemperature());

   }

   public static void read(unitree_h_one_msgs.msg.dds.IMUState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getQuaternion().length; ++i0)
      {
        	data.getQuaternion()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getGyroscope().length; ++i0)
      {
        	data.getGyroscope()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getAccelerometer().length; ++i0)
      {
        	data.getAccelerometer()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getRpy().length; ++i0)
      {
        	data.getRpy()[i0] = cdr.read_type_5();
        	
      }
      	
      data.setTemperature(cdr.read_type_1());
      	

   }

   @Override
   public final void serialize(unitree_h_one_msgs.msg.dds.IMUState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_f("quaternion", data.getQuaternion());
      ser.write_type_f("gyroscope", data.getGyroscope());
      ser.write_type_f("accelerometer", data.getAccelerometer());
      ser.write_type_f("rpy", data.getRpy());
      ser.write_type_1("temperature", data.getTemperature());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_h_one_msgs.msg.dds.IMUState data)
   {
      ser.read_type_f("quaternion", data.getQuaternion());
      ser.read_type_f("gyroscope", data.getGyroscope());
      ser.read_type_f("accelerometer", data.getAccelerometer());
      ser.read_type_f("rpy", data.getRpy());
      data.setTemperature(ser.read_type_1("temperature"));
   }

   public static void staticCopy(unitree_h_one_msgs.msg.dds.IMUState src, unitree_h_one_msgs.msg.dds.IMUState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_h_one_msgs.msg.dds.IMUState createData()
   {
      return new unitree_h_one_msgs.msg.dds.IMUState();
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
   
   public void serialize(unitree_h_one_msgs.msg.dds.IMUState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_h_one_msgs.msg.dds.IMUState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_h_one_msgs.msg.dds.IMUState src, unitree_h_one_msgs.msg.dds.IMUState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public IMUStatePubSubType newInstance()
   {
      return new IMUStatePubSubType();
   }
}
