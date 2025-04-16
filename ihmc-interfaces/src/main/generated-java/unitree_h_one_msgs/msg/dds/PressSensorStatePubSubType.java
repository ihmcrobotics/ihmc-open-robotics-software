package unitree_h_one_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PressSensorState" defined in "PressSensorState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PressSensorState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PressSensorState_.idl instead.
*
*/
public class PressSensorStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_h_one_msgs.msg.dds.PressSensorState>
{
   public static final java.lang.String name = "unitree_h_one_msgs::msg::dds_::PressSensorState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "09b7f33c03f441294e7e03e7a446f289287688c4606d8e8155e7a2e6258e9350";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_h_one_msgs.msg.dds.PressSensorState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_h_one_msgs.msg.dds.PressSensorState data) throws java.io.IOException
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

      current_alignment += ((12) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((12) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.PressSensorState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.PressSensorState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ((12) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((12) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(unitree_h_one_msgs.msg.dds.PressSensorState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getPressure().length; ++i0)
      {
        	cdr.write_type_5(data.getPressure()[i0]);	
      }

      for(int i0 = 0; i0 < data.getTemperature().length; ++i0)
      {
        	cdr.write_type_5(data.getTemperature()[i0]);	
      }

   }

   public static void read(unitree_h_one_msgs.msg.dds.PressSensorState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getPressure().length; ++i0)
      {
        	data.getPressure()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getTemperature().length; ++i0)
      {
        	data.getTemperature()[i0] = cdr.read_type_5();
        	
      }
      	

   }

   @Override
   public final void serialize(unitree_h_one_msgs.msg.dds.PressSensorState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_f("pressure", data.getPressure());
      ser.write_type_f("temperature", data.getTemperature());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_h_one_msgs.msg.dds.PressSensorState data)
   {
      ser.read_type_f("pressure", data.getPressure());
      ser.read_type_f("temperature", data.getTemperature());
   }

   public static void staticCopy(unitree_h_one_msgs.msg.dds.PressSensorState src, unitree_h_one_msgs.msg.dds.PressSensorState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_h_one_msgs.msg.dds.PressSensorState createData()
   {
      return new unitree_h_one_msgs.msg.dds.PressSensorState();
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
   
   public void serialize(unitree_h_one_msgs.msg.dds.PressSensorState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_h_one_msgs.msg.dds.PressSensorState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_h_one_msgs.msg.dds.PressSensorState src, unitree_h_one_msgs.msg.dds.PressSensorState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PressSensorStatePubSubType newInstance()
   {
      return new PressSensorStatePubSubType();
   }
}
