package us.ihmc.humanoidRobotics.communication.packets;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;

import org.junit.Test;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

@ContinuousIntegrationPlan(categories = IntegrationCategory.HEALTH)
public class PacketCodeQualityTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testAllPacketFieldsArePublic()
   {
      IHMCCommunicationKryoNetClassList classList = new IHMCCommunicationKryoNetClassList();

      for (Class<?> clazz : classList.getPacketClassList())
      {
         checkIfAllFieldsArePublic(clazz);
      }

      for (Class<?> clazz : classList.getPacketFieldList())
      {
         checkIfAllFieldsArePublic(clazz);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAllRosExportedPacketsHaveRandomConstructor()
   {
      IHMCCommunicationKryoNetClassList classList = new IHMCCommunicationKryoNetClassList();
      Set<Class> badClasses = new HashSet<>();

      for (Class<?> clazz : classList.getPacketClassList())
      {
         checkIfClassHasRandomConstructor(clazz, badClasses);
      }

      if(!badClasses.isEmpty())
      {
         System.err.println("PacketCodeQualityTest.checkIfClassHasRandomConstructor failed: The following classes do not have Random constructors:");
         for (Class badClass : badClasses)
         {
            System.err.println("- " + badClass.getCanonicalName());
         }

         fail("PacketCodeQualityTest.checkIfClassHasRandomConstructor failed. Consult Standard Error logs for list of classes without Random constructors.");
      }
   }

   private void checkIfClassHasRandomConstructor(Class<?> clazz, Set<Class> badClasses)
   {
      // Skip base class
      if(clazz == Packet.class)
      {
         return;
      }

      if(Packet.class.isAssignableFrom(clazz) && !Modifier.isAbstract(clazz.getModifiers()) && clazz.isAnnotationPresent(RosMessagePacket.class))
      {
         try
         {
            Constructor<?> constructor = clazz.getConstructor(Random.class);
         }
         catch (NoSuchMethodException e)
         {
            badClasses.add(clazz);
         }
      }
   }

   private void checkIfAllFieldsArePublic(Class<?> clazz)
   {
      if (clazz == String.class) return;
      if (clazz == ArrayList.class) return;

      for (Field field : clazz.getDeclaredFields())
      {
         if (Modifier.isStatic(field.getModifiers())) continue;
         if (Modifier.isTransient(field.getModifiers())) continue;
         if (field.isSynthetic()) continue;

         assertTrue("Class " + clazz.getCanonicalName() + " has non-public field " + field.getName() + " declared by " + field.getDeclaringClass().getCanonicalName(), Modifier.isPublic(field.getModifiers()));
         assertFalse("Class " + clazz.getCanonicalName() + " has final field " + field.getName() + " declared by " + field.getDeclaringClass().getCanonicalName(), Modifier.isFinal(field.getModifiers()));
      }
   }
}
