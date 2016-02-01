package us.ihmc.communication.net;

import java.util.ArrayList;

import com.esotericsoftware.kryo.Kryo;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TObjectIntHashMap;

public class NetClassList
{
   private final ArrayList<Class<?>> classList = new ArrayList<Class<?>>();
   private final ArrayList<Class<?>> typeList = new ArrayList<Class<?>>();

   public int uniquePacketID = 0;
   public final TObjectIntMap<Class<?>> registrationIDs = new TObjectIntHashMap<>();
   public final TIntObjectMap<Class<?>> registrationClasses = new TIntObjectHashMap<>();
   
   public NetClassList()
   {
   }

   public NetClassList(Class<?>... classes)
   {
      registerPacketClasses(classes);
   }

   public void registerPacketClass(Class<?> clazz)
   {
      classList.add(clazz);

      int id = ++uniquePacketID;
      registrationIDs.put(clazz, id);
      registrationClasses.put(id, clazz);
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
   
   
}
