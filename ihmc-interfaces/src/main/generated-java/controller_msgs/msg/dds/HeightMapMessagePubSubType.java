package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeightMapMessage" defined in "HeightMapMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeightMapMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeightMapMessage_.idl instead.
*
*/
public class HeightMapMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HeightMapMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HeightMapMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HeightMapMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HeightMapMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (30000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (30000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (30000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HeightMapMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HeightMapMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getXCells().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getYCells().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getHeights().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getXyResolution());

      cdr.write_type_6(data.getGridSizeXy());

      if(data.getXCells().size() <= 30000)
      cdr.write_type_e(data.getXCells());else
          throw new RuntimeException("x_cells field exceeds the maximum length");

      if(data.getYCells().size() <= 30000)
      cdr.write_type_e(data.getYCells());else
          throw new RuntimeException("y_cells field exceeds the maximum length");

      if(data.getHeights().size() <= 30000)
      cdr.write_type_e(data.getHeights());else
          throw new RuntimeException("heights field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setXyResolution(cdr.read_type_6());
      	
      data.setGridSizeXy(cdr.read_type_6());
      	
      cdr.read_type_e(data.getXCells());	
      cdr.read_type_e(data.getYCells());	
      cdr.read_type_e(data.getHeights());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("xy_resolution", data.getXyResolution());
      ser.write_type_6("grid_size_xy", data.getGridSizeXy());
      ser.write_type_e("x_cells", data.getXCells());
      ser.write_type_e("y_cells", data.getYCells());
      ser.write_type_e("heights", data.getHeights());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HeightMapMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setXyResolution(ser.read_type_6("xy_resolution"));
      data.setGridSizeXy(ser.read_type_6("grid_size_xy"));
      ser.read_type_e("x_cells", data.getXCells());
      ser.read_type_e("y_cells", data.getYCells());
      ser.read_type_e("heights", data.getHeights());
   }

   public static void staticCopy(controller_msgs.msg.dds.HeightMapMessage src, controller_msgs.msg.dds.HeightMapMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HeightMapMessage createData()
   {
      return new controller_msgs.msg.dds.HeightMapMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HeightMapMessage src, controller_msgs.msg.dds.HeightMapMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeightMapMessagePubSubType newInstance()
   {
      return new HeightMapMessagePubSubType();
   }
}
