package us.ihmc.humanoidRobotics.kryo;

import static org.junit.Assert.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import org.junit.Test;

import us.ihmc.communication.packets.Packet;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class IHMCCommunicationKryoNetClassListTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testAllClassesRegisteredArePackets()
   {
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      ArrayList<Class<?>> packetClassList = netClassList.getPacketClassList();

      for (Class<?> packetClass : packetClassList)
      {
         assertTrue("The class " + packetClass.getSimpleName() + " is not a packet", Packet.class.isAssignableFrom(packetClass));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testAllPacketFieldsAreRegistered()
   {
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      ArrayList<Class<?>> packetClassList = netClassList.getPacketClassList();
      Set<Class<?>> packetFieldSet = new HashSet<>(netClassList.getPacketFieldList());

      for (Class<?> packetClass : packetClassList)
      {
         for (Field field : packetClass.getDeclaredFields())
            assertAllFieldsAreInSetRecursively(field, packetFieldSet);
      }
   }

   private static void assertAllFieldsAreInSetRecursively(Field field, Set<Class<?>> setWithRegisteredFields)
   {
      if (Modifier.isStatic(field.getModifiers()))
         return;

      Class<?> typeToCheck = field.getType();

      if (typeToCheck.isPrimitive() || typeToCheck == Class.class || typeToCheck == String.class)
         return;

      assertTrue("The field " + field.getDeclaringClass().getSimpleName() + "." + field.getName() + " is not registered.",
                 setWithRegisteredFields.contains(typeToCheck));

      while (typeToCheck.isArray())
         typeToCheck = typeToCheck.getComponentType();

      if (Enum.class.isAssignableFrom(typeToCheck) || ArrayList.class.isAssignableFrom(typeToCheck))
         return;

      for (Field subField : typeToCheck.getDeclaredFields())
      {
         if (subField.getType() == field.getType())
            continue;

         assertAllFieldsAreInSetRecursively(subField, setWithRegisteredFields);
      }
   }
}
