package us.ihmc.communication.net;

import java.util.ArrayList;

import com.esotericsoftware.kryo.Kryo;

import controller_msgs.msg.dds.IMUPacket;
import controller_msgs.msg.dds.SpatialVectorMessage;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;

public class NetClassList
{
   public static interface PacketTrimmer<T>
   {
      T trim(T messageToTrim);
   }

   private final ArrayList<Class<?>> classList = new ArrayList<Class<?>>();
   private final ArrayList<Class<?>> typeList = new ArrayList<Class<?>>();

   public int uniquePacketID = 0;
   public final TObjectIntMap<Class<?>> registrationIDs = new TObjectIntHashMap<>();
   public final TIntObjectMap<Class<?>> registrationClasses = new TIntObjectHashMap<>();
   public final TIntObjectMap<PacketTrimmer<?>> classIDsToTrimmerMap = new TIntObjectHashMap<>();
   
   public NetClassList()
   {
   }

   public NetClassList(Class<?>... classes)
   {
      registerPacketClasses(classes);
   }

   public void registerPacketClass(Class<?> clazz)
   {
      registerPacketClass(clazz, null);
   }

   public void registerPacketClass(Class<?> clazz, PacketTrimmer<?> packetTrimmer)
   {
      classList.add(clazz);
      
      int id = ++uniquePacketID;
      registrationIDs.put(clazz, id);
      registrationClasses.put(id, clazz);
      classIDsToTrimmerMap.put(id, packetTrimmer);
   }

   public void registerPacketClasses(Class<?>... classes)
   {
      for (Class<?> clazz : classes)
      {
         registerPacketClass(clazz);
      }
   }

   public void registerPacketField(Class<?> type)
   {
      typeList.add(type);
   }

   public void registerPacketFields(Class<?>... types)
   {
      for (Class<?> type : types)
      {
         registerPacketField(type);
      }
   }

   public void getPacketClassList(ArrayList<Class<?>> listToPack)
   {
      listToPack.addAll(classList);
   }

   public ArrayList<Class<?>> getPacketClassList()
   {
      return classList;
   }

   public ArrayList<Class<?>> getPacketFieldList()
   {
      return typeList;
   }

   public void registerWithKryo(Kryo kryo)
   {
      kryo.addDefaultSerializer(Quaternion.class, QuaternionSerializer.class);
      kryo.addDefaultSerializer(Vector3D.class, Vector3DSerializer.class);
      kryo.addDefaultSerializer(Point3D.class, Point3DSerializer.class);
      kryo.addDefaultSerializer(IMUPacket.class, IMUPacketSerializer.class);
      kryo.addDefaultSerializer(SpatialVectorMessage.class, SpatialVectorMessageSerializer.class);
      kryo.addDefaultSerializer(IDLSequence.Object.class, IDLSequenceObjectSerializer.class);
      kryo.addDefaultSerializer(RecyclingArrayList.class, RecyclingArrayListPubSubSerializer.class);
      kryo.addDefaultSerializer(IDLSequence.Boolean.class, IDLSequenceBooleanSerializer.class);
      kryo.addDefaultSerializer(IDLSequence.Double.class, IDLSequenceDoubleSerializer.class);
      kryo.addDefaultSerializer(IDLSequence.Float.class, IDLSequenceFloatSerializer.class);
      kryo.addDefaultSerializer(IDLSequence.Integer.class, IDLSequenceIntegerSerializer.class);
      kryo.addDefaultSerializer(IDLSequence.Byte.class, IDLSequenceByteSerializer.class);
      kryo.addDefaultSerializer(IDLSequence.Long.class, IDLSequenceLongSerializer.class);

      for (Class<?> clazz : getPacketClassList())
      {
         kryo.register(clazz);
      }

      for (Class<?> type : getPacketFieldList())
      {
         kryo.register(type);
      }
   }
   
   public int getID(Class<?> clazz)
   {
      return registrationIDs.get(clazz);
   }
   
   public Class<?> getClass(int id)
   {
      return registrationClasses.get(id);
   }

   public PacketTrimmer<?> getPacketTrimmer(Class<?> clazz)
   {
      if (registrationIDs.containsKey(clazz))
      {
         int id = registrationIDs.get(clazz);
         return classIDsToTrimmerMap.get(id);
      }
      else
      {
         return null;
      }
   }
}
