package grid_map_msgs.msg.dds;

/**
* 
* Topic data type of the struct "GridMapInfo" defined in "GridMapInfo_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from GridMapInfo_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit GridMapInfo_.idl instead.
*
*/
public class GridMapInfoPubSubType implements us.ihmc.pubsub.TopicDataType<grid_map_msgs.msg.dds.GridMapInfo>
{
   public static final java.lang.String name = "grid_map_msgs::msg::dds_::GridMapInfo_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(grid_map_msgs.msg.dds.GridMapInfo data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, grid_map_msgs.msg.dds.GridMapInfo data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(grid_map_msgs.msg.dds.GridMapInfo data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(grid_map_msgs.msg.dds.GridMapInfo data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(grid_map_msgs.msg.dds.GridMapInfo data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getResolution());

      cdr.write_type_6(data.getLengthX());

      cdr.write_type_6(data.getLengthY());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getPose(), cdr);
   }

   public static void read(grid_map_msgs.msg.dds.GridMapInfo data, us.ihmc.idl.CDR cdr)
   {
      data.setResolution(cdr.read_type_6());
      	
      data.setLengthX(cdr.read_type_6());
      	
      data.setLengthY(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPose(), cdr);	

   }

   @Override
   public final void serialize(grid_map_msgs.msg.dds.GridMapInfo data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("resolution", data.getResolution());
      ser.write_type_6("length_x", data.getLengthX());
      ser.write_type_6("length_y", data.getLengthY());
      ser.write_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, grid_map_msgs.msg.dds.GridMapInfo data)
   {
      data.setResolution(ser.read_type_6("resolution"));
      data.setLengthX(ser.read_type_6("length_x"));
      data.setLengthY(ser.read_type_6("length_y"));
      ser.read_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

   }

   public static void staticCopy(grid_map_msgs.msg.dds.GridMapInfo src, grid_map_msgs.msg.dds.GridMapInfo dest)
   {
      dest.set(src);
   }

   @Override
   public grid_map_msgs.msg.dds.GridMapInfo createData()
   {
      return new grid_map_msgs.msg.dds.GridMapInfo();
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
   
   public void serialize(grid_map_msgs.msg.dds.GridMapInfo data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(grid_map_msgs.msg.dds.GridMapInfo data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(grid_map_msgs.msg.dds.GridMapInfo src, grid_map_msgs.msg.dds.GridMapInfo dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public GridMapInfoPubSubType newInstance()
   {
      return new GridMapInfoPubSubType();
   }
}
