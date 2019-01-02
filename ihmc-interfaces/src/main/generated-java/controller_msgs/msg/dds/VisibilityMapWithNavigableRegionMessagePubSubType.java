package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VisibilityMapWithNavigableRegionMessage" defined in "VisibilityMapWithNavigableRegionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VisibilityMapWithNavigableRegionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VisibilityMapWithNavigableRegionMessage_.idl instead.
*
*/
public class VisibilityMapWithNavigableRegionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::VisibilityMapWithNavigableRegionMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.PlanarRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityClusterMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 25; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.VisibilityClusterMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += controller_msgs.msg.dds.PlanarRegionMessagePubSubType.getCdrSerializedSize(data.getHomeRegion(), current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityClusterMessagePubSubType.getCdrSerializedSize(data.getHomeRegionCluster(), current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityMapMessagePubSubType.getCdrSerializedSize(data.getVisibilityMapInWorld(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getObstacleClusters().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.VisibilityClusterMessagePubSubType.getCdrSerializedSize(data.getObstacleClusters().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.PlanarRegionMessagePubSubType.write(data.getHomeRegion(), cdr);
      controller_msgs.msg.dds.VisibilityClusterMessagePubSubType.write(data.getHomeRegionCluster(), cdr);
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.write(data.getVisibilityMapInWorld(), cdr);
      if(data.getObstacleClusters().size() <= 25)
      cdr.write_type_e(data.getObstacleClusters());else
          throw new RuntimeException("obstacle_clusters field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.PlanarRegionMessagePubSubType.read(data.getHomeRegion(), cdr);	
      controller_msgs.msg.dds.VisibilityClusterMessagePubSubType.read(data.getHomeRegionCluster(), cdr);	
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.read(data.getVisibilityMapInWorld(), cdr);	
      cdr.read_type_e(data.getObstacleClusters());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("home_region", new controller_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getHomeRegion());

      ser.write_type_a("home_region_cluster", new controller_msgs.msg.dds.VisibilityClusterMessagePubSubType(), data.getHomeRegionCluster());

      ser.write_type_a("visibility_map_in_world", new controller_msgs.msg.dds.VisibilityMapMessagePubSubType(), data.getVisibilityMapInWorld());

      ser.write_type_e("obstacle_clusters", data.getObstacleClusters());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data)
   {
      ser.read_type_a("home_region", new controller_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getHomeRegion());

      ser.read_type_a("home_region_cluster", new controller_msgs.msg.dds.VisibilityClusterMessagePubSubType(), data.getHomeRegionCluster());

      ser.read_type_a("visibility_map_in_world", new controller_msgs.msg.dds.VisibilityMapMessagePubSubType(), data.getVisibilityMapInWorld());

      ser.read_type_e("obstacle_clusters", data.getObstacleClusters());
   }

   public static void staticCopy(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage src, controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage createData()
   {
      return new controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage();
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
   
   public void serialize(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage src, controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VisibilityMapWithNavigableRegionMessagePubSubType newInstance()
   {
      return new VisibilityMapWithNavigableRegionMessagePubSubType();
   }
}
