package us.ihmc.communication.net;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.reflections.Reflections;
import org.reflections.util.ConfigurationBuilder;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.ros2.ROS2TopicNameTools;

/**
 * NetClassList to make {@link KryoObjectServer} and {@link KryoObjectClient} compatible with any
 * pub-sub message. It's at the proof-of-concept stage, serialization and deserialization are
 * inefficient and need to be improved, but it should be a good start.
 */
public class PubSubNetClassListAdapter extends NetClassList
{
   private final Map<Class<?>, Serializer<?>> pubSubKryoSerializerMap = new LinkedHashMap<>();

   public PubSubNetClassListAdapter()
   {
      super();

      Reflections reflections = new Reflections(new ConfigurationBuilder().forPackages("controller_msgs.msg.dds",
                                                                                       "action_msgs.msg",
                                                                                       "actionlib_msgs.msg",
                                                                                       "builtin_interfaces.msg",
                                                                                       "diagnostic_msgs.msg",
                                                                                       "geometry_msgs.msg",
                                                                                       "nav_msgs.msg",
                                                                                       "rosgraph_msgs.msg",
                                                                                       "sensor_msgs.msg",
                                                                                       "shape_msgs.msg",
                                                                                       "std_msgs.msg",
                                                                                       "stereo_msgs.msg",
                                                                                       "tf2_msgs.msg",
                                                                                       "trajectory_msgs.msg",
                                                                                       "us.ihmc.ros2.rosidl.geometry_msgs.msg.dds"));
      List<Class<?>> pubSubMessageTypes = new ArrayList<>(reflections.getSubTypesOf(Packet.class));

      for (Class<?> pubSubMessageType : pubSubMessageTypes)
      {
         registerPacketClass(pubSubMessageType);
         try
         {
            pubSubKryoSerializerMap.put(pubSubMessageType, newPubSubSerializer(pubSubMessageType));
         }
         catch (InstantiationException | IllegalAccessException | ClassNotFoundException e)
         {
            e.printStackTrace();
         }
      }

      pubSubKryoSerializerMap.put(StereoVisionPointCloudMessage.class, new StereoVisionPointCloudMessageSerializer());
   }

   @Override
   public void registerWithKryo(Kryo kryo)
   {
      pubSubKryoSerializerMap.forEach((type, ser) -> kryo.addDefaultSerializer(type, ser));

      super.registerWithKryo(kryo);
   }

   public static <T> Serializer<T> newPubSubSerializer(Class<T> messageType) throws InstantiationException, IllegalAccessException, ClassNotFoundException
   {
      @SuppressWarnings("unchecked")
      TopicDataType<T> topicDataType = (TopicDataType<T>) Class.forName(messageType.getName() + "PubSubType").newInstance();
      SerializedPayload serializedPayload = new SerializedPayload(topicDataType.getTypeSize());

      return new Serializer<T>()
      {
         @Override
         public void write(Kryo kryo, Output output, T object)
         {
            try
            {
               serializedPayload.getData().clear();
               topicDataType.serialize(object, serializedPayload);

               for (int i = 0; i < serializedPayload.getLength(); i++)
               {
                  output.writeByte(serializedPayload.getData().get());
               }
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }

         @Override
         public T read(Kryo kryo, Input input, Class<? extends T> type)
         {
            try
            {
               T message = ROS2TopicNameTools.newMessageInstance(messageType);
               serializedPayload.getData().clear();
               int length = input.limit() - input.position();
               for (int i = 0; i < length; i++)
                  serializedPayload.getData().put(input.readByte());
               serializedPayload.getData().flip();
               serializedPayload.setLength(length);
               topicDataType.deserialize(serializedPayload, message);
               return message;
            }
            catch (Exception e)
            {
               e.printStackTrace();
               return null;
            }
         }
      };
   }

   public static void main(String[] args)
   {
      new PubSubNetClassListAdapter();
   }
}
