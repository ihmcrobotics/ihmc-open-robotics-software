package us.ihmc.avatar.ros;

import org.junit.Test;
import org.reflections.Reflections;
import org.ros.internal.message.Message;

import us.ihmc.avatar.ros.IHMCROSTranslationRuntimeTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.io.printing.PrintTools;

import java.lang.reflect.*;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;

import static org.junit.Assert.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class IHMCROSTranslationRuntimeToolsTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 24.4)
   @Test(timeout = 120000)
   public void testBidirectionalConversionWithRandomConstructors()
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
      Random random = new Random(1976L);

      for(int i = 0; i < 5000; i++)
      {
         int packetsFailed = 0;
         for (Class<?> concreteType : concreteTypes)
         {
            Constructor<?> randomConstructor = null;
            try
            {
               randomConstructor = concreteType.getConstructor(Random.class);
               ihmcMessage = (Packet<?>) randomConstructor.newInstance(random);
               rosMessage = IHMCROSTranslationRuntimeTools.convertToRosMessage(ihmcMessage);
               Packet packet = IHMCROSTranslationRuntimeTools.convertToIHMCMessage(rosMessage);
               assertTrue("Problem with packet " + concreteType + ". \n" + ihmcMessage + ", \n" + packet, packet.epsilonEquals(ihmcMessage, 0.1));
            }
            catch (Exception e)
            {
               PrintTools.error("Conversion failed: " + e.getMessage());
               PrintTools.error("Message type: " + concreteType);
               e.printStackTrace();
               ++packetsFailed;
            }
         }
         assertTrue("Conversion(s) failed for " + packetsFailed + " packets.", packetsFailed < 1);
      }
   }
}
