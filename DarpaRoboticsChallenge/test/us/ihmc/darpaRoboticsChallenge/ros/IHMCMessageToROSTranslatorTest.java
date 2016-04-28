package us.ihmc.darpaRoboticsChallenge.ros;

import org.junit.Test;
import org.reflections.Reflections;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;

import javax.vecmath.Quat4d;
import javax.vecmath.Tuple4d;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.lang.reflect.Type;
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

      for (Class<?> concreteType : concreteTypes)
      {
         Constructor<?> defaultConstructor = null;
         try
         {
            defaultConstructor = concreteType.getConstructor();
            Packet<?> ihmcMessage = (Packet<?>) defaultConstructor.newInstance();
            IHMCMessageToROSTranslator.convertToRosMessage(ihmcMessage);
         }
         catch (Exception e)
         {
            System.out.println("Failed converting message of type : " + concreteType);
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