package us.ihmc.humanoidRobotics.communication.util;

import static org.junit.Assert.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Set;
import java.util.TreeSet;

import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.idl.PreallocatedList;

public class MessageTrimmingToolsTest
{
   @Test
   public void testTrimmerForAllTrimmablePacket() throws Exception
   {
      boolean verbose = true;
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();

      Set<Class<?>> typesMissingTrimmer = new TreeSet<>((o1, o2) -> o1.getSimpleName().compareTo(o2.getSimpleName()));

      for (Class<?> packetType : netClassList.getPacketClassList())
      {
         boolean trimmable = isTypeTrimmable(packetType);

         if (trimmable)
         {
            boolean trimmerExists = hasMessageTrimmer(packetType);
            if (!trimmerExists)
               typesMissingTrimmer.add(packetType);
         }
      }

      if (verbose)
      {
         if (!typesMissingTrimmer.isEmpty())
         {
            PrintTools.error("Some packets are missing a MessageTrimmer:");
            typesMissingTrimmer.stream().map(Class::getSimpleName).forEach(PrintTools::error);
         }
      }

      assertTrue("Some packets are missing a MessageTrimmer.", typesMissingTrimmer.isEmpty());
   }

   private boolean hasMessageTrimmer(Class<?> packetType)
   {
      String methodName = "create" + packetType.getSimpleName() + "Trimmer";
      try
      {
         Method method = MessageTrimmingTools.class.getDeclaredMethod(methodName);
         if (method.getReturnType() == NetClassList.PacketTrimmer.class)
            return true;
      }
      catch (NoSuchMethodException | SecurityException e)
      {
      }
      return false;
   }

   private boolean isTypeTrimmable(Class<?> typeToCheck)
   {
      if (typeToCheck.isPrimitive())
         return false;

      for (Field field : typeToCheck.getDeclaredFields())
      {
         if (PreallocatedList.class == field.getType())
            return true;
         if (isTypeTrimmable(field.getType()))
            return true;
      }
      return false;
   }
}
