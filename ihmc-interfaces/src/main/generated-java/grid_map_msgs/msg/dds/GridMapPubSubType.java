package grid_map_msgs.msg.dds;

/**
 *
 * Topic data type of the struct "GridMap" defined in "GridMap_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from GridMap_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit GridMap_.idl instead.
 *
 */
public class GridMapPubSubType implements us.ihmc.pubsub.TopicDataType<grid_map_msgs.msg.dds.GridMap>
{
    public static final java.lang.String name = "grid_map_msgs::msg::dds_::GridMap_";

    private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
    private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    @Override
    public void serialize(grid_map_msgs.msg.dds.GridMap data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
    {
        serializeCDR.serialize(serializedPayload);
        write(data, serializeCDR);
        serializeCDR.finishSerialize();
    }

    @Override
    public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, grid_map_msgs.msg.dds.GridMap data) throws java.io.IOException
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

        current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);

        current_alignment += grid_map_msgs.msg.dds.GridMapInfoPubSubType.getMaxCdrSerializedSize(current_alignment);

        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
    {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
    }
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
    {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
    }
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
    {
        current_alignment += std_msgs.msg.dds.Float32MultiArrayPubSubType.getMaxCdrSerializedSize(current_alignment);}
        current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

        current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


        return current_alignment - initial_alignment;
    }

    public final static int getCdrSerializedSize(grid_map_msgs.msg.dds.GridMap data)
    {
        return getCdrSerializedSize(data, 0);
    }

    public final static int getCdrSerializedSize(grid_map_msgs.msg.dds.GridMap data, int current_alignment)
    {
        int initial_alignment = current_alignment;

        current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

        current_alignment += grid_map_msgs.msg.dds.GridMapInfoPubSubType.getCdrSerializedSize(data.getInfo(), current_alignment);

        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
        for(int i0 = 0; i0 < data.getLayers().size(); ++i0)
        {
            current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getLayers().get(i0).length() + 1;
        }
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
        for(int i0 = 0; i0 < data.getBasicLayers().size(); ++i0)
        {
            current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getBasicLayers().get(i0).length() + 1;
        }
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
        for(int i0 = 0; i0 < data.getData().size(); ++i0)
        {
            current_alignment += std_msgs.msg.dds.Float32MultiArrayPubSubType.getCdrSerializedSize(data.getData().get(i0), current_alignment);}

        current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


        current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



        return current_alignment - initial_alignment;
    }

    public static void write(grid_map_msgs.msg.dds.GridMap data, us.ihmc.idl.CDR cdr)
    {
        std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
        grid_map_msgs.msg.dds.GridMapInfoPubSubType.write(data.getInfo(), cdr);
        if(data.getLayers().size() <= 100)
            cdr.write_type_e(data.getLayers());else
            throw new RuntimeException("layers field exceeds the maximum length");

        if(data.getBasicLayers().size() <= 100)
            cdr.write_type_e(data.getBasicLayers());else
            throw new RuntimeException("basic_layers field exceeds the maximum length");

        if(data.getData().size() <= 100)
            cdr.write_type_e(data.getData());else
            throw new RuntimeException("data field exceeds the maximum length");

        cdr.write_type_3(data.getOuterStartIndex());

        cdr.write_type_3(data.getInnerStartIndex());

    }

    public static void read(grid_map_msgs.msg.dds.GridMap data, us.ihmc.idl.CDR cdr)
    {
        std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);
        grid_map_msgs.msg.dds.GridMapInfoPubSubType.read(data.getInfo(), cdr);
        cdr.read_type_e(data.getLayers());
        cdr.read_type_e(data.getBasicLayers());
        cdr.read_type_e(data.getData());
        data.setOuterStartIndex(cdr.read_type_3());

        data.setInnerStartIndex(cdr.read_type_3());


    }

    @Override
    public final void serialize(grid_map_msgs.msg.dds.GridMap data, us.ihmc.idl.InterchangeSerializer ser)
    {
        ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

        ser.write_type_a("info", new grid_map_msgs.msg.dds.GridMapInfoPubSubType(), data.getInfo());

        ser.write_type_e("layers", data.getLayers());
        ser.write_type_e("basic_layers", data.getBasicLayers());
        ser.write_type_e("data", data.getData());
        ser.write_type_3("outer_start_index", data.getOuterStartIndex());
        ser.write_type_3("inner_start_index", data.getInnerStartIndex());
    }

    @Override
    public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, grid_map_msgs.msg.dds.GridMap data)
    {
        ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

        ser.read_type_a("info", new grid_map_msgs.msg.dds.GridMapInfoPubSubType(), data.getInfo());

        ser.read_type_e("layers", data.getLayers());
        ser.read_type_e("basic_layers", data.getBasicLayers());
        ser.read_type_e("data", data.getData());
        data.setOuterStartIndex(ser.read_type_3("outer_start_index"));
        data.setInnerStartIndex(ser.read_type_3("inner_start_index"));
    }

    public static void staticCopy(grid_map_msgs.msg.dds.GridMap src, grid_map_msgs.msg.dds.GridMap dest)
    {
        dest.set(src);
    }

    @Override
    public grid_map_msgs.msg.dds.GridMap createData()
    {
        return new grid_map_msgs.msg.dds.GridMap();
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

    public void serialize(grid_map_msgs.msg.dds.GridMap data, us.ihmc.idl.CDR cdr)
    {
        write(data, cdr);
    }

    public void deserialize(grid_map_msgs.msg.dds.GridMap data, us.ihmc.idl.CDR cdr)
    {
        read(data, cdr);
    }

    public void copy(grid_map_msgs.msg.dds.GridMap src, grid_map_msgs.msg.dds.GridMap dest)
    {
        staticCopy(src, dest);
    }

    @Override
    public GridMapPubSubType newInstance()
    {
        return new GridMapPubSubType();
    }
}