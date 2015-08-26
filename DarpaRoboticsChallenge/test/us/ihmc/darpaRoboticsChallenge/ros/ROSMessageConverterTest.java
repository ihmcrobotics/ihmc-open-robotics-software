package us.ihmc.darpaRoboticsChallenge.ros;

/**
 * Created by agrabertilton on 10/8/14.
 * Test for ROSMessageConverter
 */
import static org.junit.Assert.assertEquals;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;

import org.junit.Test;
import org.ros.internal.message.Message;

import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.utilities.io.printing.PrintTools;

@BambooPlan(planType = BambooPlanType.Fast)
public class ROSMessageConverterTest
{
   private static final int NUMBER_OF_TIMES_TO_RUN_EACH_PACKET_TYPE = 10000;

   @EstimatedDuration(duration = 20.0)
   @Test(timeout=300000)
   public void AllPacketTest()
   {
      PrintTools.info(this, "If this test fails, run ROSMessageGenerator then MessageAndServiceInterfaceGenerator.\n\tSee Confluence for more details (https://confluence.ihmc.us/display/HOWTO/How+to+Generate+Code+after+Packet+Change)");

      Map<String, Boolean> packetTranslationResults = new LinkedHashMap<>();
      
      Random random = new Random();
      for (Class clazz : IHMCRosApiMessageMap.PACKET_LIST)
      {
         System.out.println("Testing " + clazz.getSimpleName());
         for (int i = 0; i < NUMBER_OF_TIMES_TO_RUN_EACH_PACKET_TYPE; i++)
         {
            boolean result = testPacketTranslation(clazz, random);
            
            if (!result)
            {
               packetTranslationResults.put(clazz.getSimpleName(), result);
               break;
            }
         }
      }
      
      assertEquals("Packets not translated correctly: " + packetTranslationResults, 0, packetTranslationResults.keySet().size());
   }

//   @EstimatedDuration(duration = 0.1)
//   @Test(timeout=300000)
//   public void SinglePacketTest()
//   {      
//      Random random = new Random();
//      Class clazz = 
//      for (int i = 0; i < NUMBER_OF_TIMES_TO_RUN_EACH_PACKET_TYPE; i++)
//      {
//         assertTrue(testPacketTranslation(clazz, random));
//      }
//   }

   private static <T extends IHMCRosApiPacket> boolean testPacketTranslation(Class<T> clazz, Random random)
   {
      T untranslated = null;
      Message translatedMessage;
      T translated = null;
      try
      {
         Constructor constructor = clazz.getDeclaredConstructor(Random.class);
         untranslated = (T) constructor.newInstance(random);

         translatedMessage = DRCROSMessageConverter.convertToRosMessage(untranslated);
         translated = (T) DRCROSMessageConverter.convertToPacket(translatedMessage);

         boolean packetConversionResult;
         try
         {
            packetConversionResult = untranslated.rosConversionEpsilonEquals(translated, 0.0003);
         }
         catch (ClassCastException e)
         {
            PrintTools.error("ROSMessageConverter.convertToPacket returns incorrect type for " + clazz.getSimpleName());
            
            e.printStackTrace();
            return false;
         }
         
         if(!packetConversionResult)
         {
            System.out.println(clazz.getSimpleName() + " failed!");
            System.out.println(untranslated.toString());
            System.out.println(translated.toString());
         }
         return packetConversionResult;
      }
      catch (InstantiationException | IllegalAccessException | NoSuchMethodException | InvocationTargetException e)
      {
         System.out.println("Please make sure you generated the messages and interfaces, which may be the issue");
         e.printStackTrace();
      }

      return false;
   }
}
