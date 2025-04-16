package unitree_h_one_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BmsState" defined in "BmsState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BmsState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BmsState_.idl instead.
*
*/
public class BmsStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_h_one_msgs.msg.dds.BmsState>
{
   public static final java.lang.String name = "unitree_h_one_msgs::msg::dds_::BmsState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ec8b484cd872cf368290ad96300269178e8de27665977b3aa1a4bcffd8c5951c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_h_one_msgs.msg.dds.BmsState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_h_one_msgs.msg.dds.BmsState data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((12) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.BmsState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.BmsState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((12) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);
      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_h_one_msgs.msg.dds.BmsState data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getVersionHigh());

      cdr.write_type_9(data.getVersionLow());

      cdr.write_type_9(data.getFn());

      cdr.write_type_2(data.getCurrent());

      cdr.write_type_9(data.getSoc());

      cdr.write_type_9(data.getSoh());

      for(int i0 = 0; i0 < data.getTemperature().length; ++i0)
      {
        	cdr.write_type_1(data.getTemperature()[i0]);	
      }

      cdr.write_type_3(data.getCycle());

      cdr.write_type_3(data.getManufacturerDate());

   }

   public static void read(unitree_h_one_msgs.msg.dds.BmsState data, us.ihmc.idl.CDR cdr)
   {
      data.setVersionHigh(cdr.read_type_9());
      	
      data.setVersionLow(cdr.read_type_9());
      	
      data.setFn(cdr.read_type_9());
      	
      data.setCurrent(cdr.read_type_2());
      	
      data.setSoc(cdr.read_type_9());
      	
      data.setSoh(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getTemperature().length; ++i0)
      {
        	data.getTemperature()[i0] = cdr.read_type_1();
        	
      }
      	
      data.setCycle(cdr.read_type_3());
      	
      data.setManufacturerDate(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(unitree_h_one_msgs.msg.dds.BmsState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("version_high", data.getVersionHigh());
      ser.write_type_9("version_low", data.getVersionLow());
      ser.write_type_9("fn", data.getFn());
      ser.write_type_2("current", data.getCurrent());
      ser.write_type_9("soc", data.getSoc());
      ser.write_type_9("soh", data.getSoh());
      ser.write_type_f("temperature", data.getTemperature());
      ser.write_type_3("cycle", data.getCycle());
      ser.write_type_3("manufacturer_date", data.getManufacturerDate());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_h_one_msgs.msg.dds.BmsState data)
   {
      data.setVersionHigh(ser.read_type_9("version_high"));
      data.setVersionLow(ser.read_type_9("version_low"));
      data.setFn(ser.read_type_9("fn"));
      data.setCurrent(ser.read_type_2("current"));
      data.setSoc(ser.read_type_9("soc"));
      data.setSoh(ser.read_type_9("soh"));
      ser.read_type_f("temperature", data.getTemperature());
      data.setCycle(ser.read_type_3("cycle"));
      data.setManufacturerDate(ser.read_type_3("manufacturer_date"));
   }

   public static void staticCopy(unitree_h_one_msgs.msg.dds.BmsState src, unitree_h_one_msgs.msg.dds.BmsState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_h_one_msgs.msg.dds.BmsState createData()
   {
      return new unitree_h_one_msgs.msg.dds.BmsState();
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
   
   public void serialize(unitree_h_one_msgs.msg.dds.BmsState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_h_one_msgs.msg.dds.BmsState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_h_one_msgs.msg.dds.BmsState src, unitree_h_one_msgs.msg.dds.BmsState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BmsStatePubSubType newInstance()
   {
      return new BmsStatePubSubType();
   }
}
