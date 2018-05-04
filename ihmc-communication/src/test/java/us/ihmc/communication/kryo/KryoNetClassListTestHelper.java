package us.ihmc.communication.kryo;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.IDLSequence;
import us.ihmc.pubsub.TopicDataType;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.assertTrue;

public class KryoNetClassListTestHelper
{
   public static void testAllClassesRegisteredArePackets(NetClassList netClassList)
   {
      ArrayList<Class<?>> packetClassList = netClassList.getPacketClassList();

      for (Class<?> packetClass : packetClassList)
      {
         assertTrue("The class " + packetClass.getSimpleName() + " is not a packet", Packet.class.isAssignableFrom(packetClass));
      }
   }

   public static void testAllPacketFieldsAreRegistered(NetClassList netClassList)
         throws NoSuchFieldException, IllegalAccessException, InstantiationException
   {
      ArrayList<Class<?>> packetClassList = netClassList.getPacketClassList();
      Set<Class<?>> packetFieldSet = new HashSet<>(netClassList.getPacketFieldList());
      packetClassList.remove(Packet.class);

      for (Class<?> packetClass : packetClassList)
      {
         for (Field field : packetClass.getDeclaredFields())
         {
            assertAllFieldsAreInSetRecursively(packetClass.newInstance(), field, packetFieldSet);
         }
      }
   }

   @SuppressWarnings("rawtypes")
   private static void assertAllFieldsAreInSetRecursively(Object holder, Field field, Set<Class<?>> setWithRegisteredFields)
         throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException, InstantiationException
   {
      if (Modifier.isStatic(field.getModifiers()) || field.getType().isPrimitive())
         return;

      Class<?> typeToCheck = field.getType();
      field.setAccessible(true);
      Object fieldInstance;

      if (holder != null)
      {
         fieldInstance = field.get(holder);
      }
      else if (Packet.class.isAssignableFrom(field.getDeclaringClass()))
      {
         holder = field.getDeclaringClass().newInstance();
         fieldInstance = field.get(holder);
      }
      else if (Packet.class.isAssignableFrom(field.getType()))
      {
         fieldInstance = field.getType().newInstance();
      }
      else
      {
         fieldInstance = null;
      }

      if (typeToCheck.isPrimitive() || typeToCheck == Class.class || typeToCheck == String.class)
         return;

      if (IDLSequence.Object.class.isAssignableFrom(typeToCheck))
      {
         if (fieldInstance == null)
            return;
         Class<? extends TopicDataType> topicDataType = ((IDLSequence.Object) fieldInstance).getTopicDataType().getClass();
         assertTrue("The class " + topicDataType.getSimpleName() + " is not registered.", setWithRegisteredFields.contains(topicDataType));

         Field listClassField = typeToCheck.getSuperclass().getDeclaredField("clazz");
         listClassField.setAccessible(true);
         typeToCheck = (Class<?>) listClassField.get(fieldInstance);
         fieldInstance = null;

         { // Also need to check the array version of the class
            Class<?> arrayVersion = Array.newInstance(typeToCheck, 1).getClass();
            assertTrue("The class " + arrayVersion.getSimpleName() + " is not registered.", setWithRegisteredFields.contains(arrayVersion));
         }
      }

      assertTrue("The field " + field.getDeclaringClass().getSimpleName() + "." + field.getName() + " is not registered, unregistered type: " + typeToCheck + ".",
                 setWithRegisteredFields.contains(typeToCheck));

      while (typeToCheck.isArray())
         typeToCheck = typeToCheck.getComponentType();

      if (Enum.class.isAssignableFrom(typeToCheck))
         return;
      if (ArrayList.class.isAssignableFrom(typeToCheck))
         return;

      for (Field subField : typeToCheck.getDeclaredFields())
      {
         if (subField.getType() == field.getType())
            continue;

         assertAllFieldsAreInSetRecursively(fieldInstance, subField, setWithRegisteredFields);
      }
   }
}
