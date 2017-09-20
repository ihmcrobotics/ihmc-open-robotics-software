package us.ihmc.avatar.ros;

import java.lang.reflect.Constructor;
import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

import org.reflections.Reflections;

import us.ihmc.communication.ros.generators.RosCustomGenerator;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.utilities.ros.ROSMessageFileCreator;

public class DRCROSMessageGenerator
{
   public static void generate() throws Exception
   {
      ROSMessageFileCreator messageGenerator = new ROSMessageFileCreator(true);

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<?>> concreteTypes = new HashSet<>();
      for (Class<?> aClass : reflections.getTypesAnnotatedWith(RosMessagePacket.class))
      {
         if(!Modifier.isAbstract(aClass.getModifiers()))
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
