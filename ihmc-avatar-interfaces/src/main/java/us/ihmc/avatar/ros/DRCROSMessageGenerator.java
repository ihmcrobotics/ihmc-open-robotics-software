package us.ihmc.avatar.ros;

import java.lang.reflect.Constructor;
import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

import org.reflections.Reflections;

import us.ihmc.communication.ros.generators.RosCustomGenerator;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.utilities.ros.ROSMessageFileCreator;
import us.ihmc.utilities.ros.bootstrap.MessageAndServiceInterfaceGenerator;

/**
 * Regenerate the ROS messages from the IHMC Java packets.
 * <p>
 * This is the first step when updating the ROS API. Next step is to run
 * {@link MessageAndServiceInterfaceGenerator} with program arguments that should be something like
 * : "../ihmc-ros-tools/generated-src ROSMessagesAndServices"
 * </p>
 */
public class DRCROSMessageGenerator
{
   public static void generate() throws Exception
   {
      ROSMessageFileCreator messageGenerator = new ROSMessageFileCreator(true);

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<?>> concreteTypes = new HashSet<>();
      for (Class<?> aClass : reflections.getTypesAnnotatedWith(RosMessagePacket.class))
      {
         if (!Modifier.isAbstract(aClass.getModifiers()))
         {
            concreteTypes.add(aClass);
         }
      }

      for (Class<? extends RosCustomGenerator> aClass : reflections.getSubTypesOf(RosCustomGenerator.class))
      {
         Constructor<? extends RosCustomGenerator> constructor = aClass.getConstructor();
         RosCustomGenerator rosCustomGenerator = constructor.newInstance();
         messageGenerator.createNewRosMessageFromGenerator(rosCustomGenerator, true);
      }

      for (Class<?> concreteType : concreteTypes)
      {
         messageGenerator.createNewRosMessage(concreteType, true);
      }
   }

   public static void main(String... args) throws Exception
   {
      generate();
   }
}
