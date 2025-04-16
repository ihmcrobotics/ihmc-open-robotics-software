package unitree_h_one_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MainBoardState" defined in "MainBoardState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MainBoardState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MainBoardState_.idl instead.
*
*/
public class MainBoardStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_h_one_msgs.msg.dds.MainBoardState>
{
   public static final java.lang.String name = "unitree_h_one_msgs::msg::dds_::MainBoardState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ce97cfa0dcac5c68d4938ff84edc59b955d94a58dda0cbbd6e1f8c8bef38ffb1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_h_one_msgs.msg.dds.MainBoardState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_h_one_msgs.msg.dds.MainBoardState data) throws java.io.IOException
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

      current_alignment += ((6) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.MainBoardState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.MainBoardState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ((6) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);
      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(unitree_h_one_msgs.msg.dds.MainBoardState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getTemperature().length; ++i0)
      {
        	cdr.write_type_1(data.getTemperature()[i0]);	
      }

      for(int i0 = 0; i0 < data.getValue().length; ++i0)
      {
        	cdr.write_type_5(data.getValue()[i0]);	
      }

   }

   public static void read(unitree_h_one_msgs.msg.dds.MainBoardState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getTemperature().length; ++i0)
      {
        	data.getTemperature()[i0] = cdr.read_type_1();
        	
      }
      	
      for(int i0 = 0; i0 < data.getValue().length; ++i0)
      {
        	data.getValue()[i0] = cdr.read_type_5();
        	
      }
      	

   }

   @Override
   public final void serialize(unitree_h_one_msgs.msg.dds.MainBoardState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_f("temperature", data.getTemperature());
      ser.write_type_f("value", data.getValue());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_h_one_msgs.msg.dds.MainBoardState data)
   {
      ser.read_type_f("temperature", data.getTemperature());
      ser.read_type_f("value", data.getValue());
   }

   public static void staticCopy(unitree_h_one_msgs.msg.dds.MainBoardState src, unitree_h_one_msgs.msg.dds.MainBoardState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_h_one_msgs.msg.dds.MainBoardState createData()
   {
      return new unitree_h_one_msgs.msg.dds.MainBoardState();
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
   
   public void serialize(unitree_h_one_msgs.msg.dds.MainBoardState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_h_one_msgs.msg.dds.MainBoardState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_h_one_msgs.msg.dds.MainBoardState src, unitree_h_one_msgs.msg.dds.MainBoardState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MainBoardStatePubSubType newInstance()
   {
      return new MainBoardStatePubSubType();
   }
}
