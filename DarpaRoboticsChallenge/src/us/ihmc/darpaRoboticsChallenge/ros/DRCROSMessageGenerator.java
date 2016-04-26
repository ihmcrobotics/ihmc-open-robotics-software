package us.ihmc.darpaRoboticsChallenge.ros;

import org.reflections.Reflections;
import us.ihmc.communication.annotations.ros.RosMessagePacket;
import us.ihmc.utilities.ros.ROSMessageGenerator;

import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

public class DRCROSMessageGenerator
{
   public static void generate() throws Exception
   {
      ROSMessageGenerator messageGenerator = new ROSMessageGenerator(true);

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
         messageGenerator.createNewRosMessage(concreteType, true);
      }
   }

   public static void main(String... args) throws Exception
   {
      generate();
   }
}
