package us.ihmc.communication.packets;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = TestPlanTarget.CodeQuality)
public class PacketCodeQualityTest
{
	@DeployableTestMethod(estimatedDuration = 0.1)
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
