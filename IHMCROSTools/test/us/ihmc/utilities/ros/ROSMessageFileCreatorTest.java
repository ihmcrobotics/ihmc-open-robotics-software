package us.ihmc.utilities.ros;

import static org.junit.Assert.fail;

import java.util.Set;

import org.junit.Test;

import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;

/**
 * <p>
 * Notes to future refactorers: There is a reason that this test is not in IHMCROSTools even though it is testing
 * a class that lives there.
 * </p>
 *
 * <p>
 * Because this test works by gathering information from the working java classpath, it must live at a higher level than
 * the packets it is testing.
 * </p>
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ROSMessageFileCreatorTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAllExportedPacketsWithTopicsAreFormattedCorrectly()
   {
      Set<Class<?>> rosMessagePacketAnnotatedClasses = GenericROSTranslationTools.getIHMCCoreRosMessagePacketAnnotatedClasses();
      for (Class<?> rosMessagePacketAnnotatedClass : rosMessagePacketAnnotatedClasses)
      {
         RosMessagePacket annotation = rosMessagePacketAnnotatedClass.getAnnotation(RosMessagePacket.class);
         String topicString = annotation.topic();
         switch (topicString)
         {
         case RosMessagePacket.NO_CORRESPONDING_TOPIC_STRING:
            break;
         default:
            if (!(topicString.startsWith("/") && topicString.length() > 1))
            {
               fail("Topic string for packet " + rosMessagePacketAnnotatedClass.getSimpleName()
                     + " is not formatted correctly! Topics must start with \"/\" and must be at least one letter!\nIf this packet does not have a topic associated with it, do not set the topic.");
            }
         }
      }
   }
}