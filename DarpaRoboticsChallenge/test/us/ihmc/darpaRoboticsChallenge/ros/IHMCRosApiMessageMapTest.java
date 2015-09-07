package us.ihmc.darpaRoboticsChallenge.ros;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

@BambooPlan(planType = BambooPlanType.Fast)
public class IHMCRosApiMessageMapTest
{
   final Set<Class> packetClasses = new HashSet<>();
   final Set<Class> inputPacketClasses = new HashSet<>();
   final Set<Class> outputPacketClasses = new HashSet<>();

   @Before
   public void setupCollections()
   {
      Collections.addAll(packetClasses, IHMCRosApiMessageMap.PACKET_LIST);
      Collections.addAll(inputPacketClasses, IHMCRosApiMessageMap.INPUT_PACKET_LIST);
      Collections.addAll(outputPacketClasses, IHMCRosApiMessageMap.OUTPUT_PACKET_LIST);
      
      assertFalse(packetClasses.isEmpty());
      assertFalse(inputPacketClasses.isEmpty());
      assertFalse(outputPacketClasses.isEmpty());
   }
   
	@EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testPacketsAreClassified()
   {
      assertEquals(IHMCRosApiMessageMap.PACKET_TO_TOPIC_MAP.keySet().size(), inputPacketClasses.size() + outputPacketClasses.size());

      for(Class clazz : IHMCRosApiMessageMap.PACKET_TO_TOPIC_MAP.keySet())
      {
         assertTrue(inputPacketClasses.contains(clazz) || outputPacketClasses.contains(clazz));
      }
   }
}
