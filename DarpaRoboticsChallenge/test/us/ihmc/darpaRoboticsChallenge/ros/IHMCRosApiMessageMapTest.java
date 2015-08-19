package us.ihmc.darpaRoboticsChallenge.ros;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertFalse;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.ros.IHMCRosApiMessageMap;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

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
   
   @EstimatedDuration(duration = 0.01)
   @Test(timeout = 1000)
   public void testPacketsAreClassified()
   {
      assertEquals(packetClasses.size(), inputPacketClasses.size() + outputPacketClasses.size());
      
      for(Class clazz : IHMCRosApiMessageMap.PACKET_LIST)
      {
         assertTrue(inputPacketClasses.contains(clazz) || outputPacketClasses.contains(clazz));
      }
   }
   
   @EstimatedDuration(duration = 0.01)
   @Test(timeout = 1000)
   public void testPacketsHaveTopics()
   {
      Set<Class> packetsWithRosTopics = IHMCRosApiMessageMap.PACKET_TO_TOPIC_MAP.keySet();
      assertTrue(packetsWithRosTopics.containsAll(packetClasses));
   }
}
