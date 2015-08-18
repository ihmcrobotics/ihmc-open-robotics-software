package us.ihmc.communication.net;

import java.util.ArrayList;

import com.esotericsoftware.kryo.Kryo;

public class NetClassList
{
   private final ArrayList<Class<?>> classList = new ArrayList<Class<?>>();
   private final ArrayList<Class<?>> typeList = new ArrayList<Class<?>>();

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
}
