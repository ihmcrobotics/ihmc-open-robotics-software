package us.ihmc.darpaRoboticsChallenge.ros;

import org.junit.Test;
import org.reflections.Reflections;
import org.ros.internal.message.Message;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

import javax.vecmath.Quat4d;
import javax.vecmath.Tuple4d;
import java.lang.reflect.*;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class IHMCMessageToROSTranslatorTest
{

   @Test
   public void testConvertToRosMessageWithDefaultConstructors()
   {
      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<?>> concreteTypes = new HashSet<>();
      for (Class<?> aClass : reflections.getTypesAnnotatedWith(RosMessagePacket.class))
      {
         if(!Modifier.isAbstract(aClass.getModifiers()))
         {
            concreteTypes.add(aClass);
         }
      }

      Message rosMessage = null;
      Packet<?> ihmcMessage = null;

      for (Class<?> concreteType : concreteTypes)
      {
         Constructor<?> defaultConstructor = null;
         try
         {
            defaultConstructor = concreteType.getConstructor();
            ihmcMessage = (Packet<?>) defaultConstructor.newInstance();
            rosMessage = IHMCMessageToROSTranslator.convertToRosMessage(ihmcMessage);
         }
         catch (Exception e)
         {
            System.out.println("Conversion from IHMC Message -> ROS Message failed!");
            System.out.println("Message type: " + concreteType);
            e.printStackTrace();
            fail();
         }
         try
         {
            Packet<?> packet = IHMCMessageToROSTranslator.convertToIHMCMessage(rosMessage);
         }
         catch (Exception e)
         {
            System.out.println("Conversion from ROS Message -> IHMC Message failed!");
            System.out.println("Message type: " + concreteType);
            e.printStackTrace();
            fail();
         }

      }
   }

   @Test
   public void testConvertToIHMCMessageWithDefaultConstructors() throws Exception
   {

   }
}