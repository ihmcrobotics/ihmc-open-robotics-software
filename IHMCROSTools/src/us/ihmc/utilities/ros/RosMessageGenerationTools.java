package us.ihmc.utilities.ros;

import org.reflections.ReflectionUtils;

import javax.vecmath.*;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Collections;
import java.util.HashMap;
import java.util.Set;

/**
 * Helper function to document Enums that do not depend on DocumentedEnum
 *
 */
public class RosMessageGenerationTools
{
   private static final HashMap<Class<?>, String> javaClassToRosMessageTypeMap = new HashMap<>();

   /* Initialize the class to message type map */
   static
   {
      javaClassToRosMessageTypeMap.put(byte.class, "int8");
      javaClassToRosMessageTypeMap.put(Byte.class, "int8");

      javaClassToRosMessageTypeMap.put(short.class, "int16");
      javaClassToRosMessageTypeMap.put(Short.class, "int16");

      javaClassToRosMessageTypeMap.put(int.class, "int32");
      javaClassToRosMessageTypeMap.put(Integer.class, "int32");

      javaClassToRosMessageTypeMap.put(long.class, "int64");
      javaClassToRosMessageTypeMap.put(Long.class, "int64");

      javaClassToRosMessageTypeMap.put(float.class, "float32");
      javaClassToRosMessageTypeMap.put(Float.class, "float32");

      javaClassToRosMessageTypeMap.put(double.class, "float64");
      javaClassToRosMessageTypeMap.put(Double.class, "float64");

      javaClassToRosMessageTypeMap.put(boolean.class, "bool");
      javaClassToRosMessageTypeMap.put(Boolean.class, "bool");

      javaClassToRosMessageTypeMap.put(char.class, "uint8");
      javaClassToRosMessageTypeMap.put(Character.class, "uint8");

      javaClassToRosMessageTypeMap.put(String.class, "string");

      javaClassToRosMessageTypeMap.put(Quat4d.class, "geometry_msgs/Quaternion");
      javaClassToRosMessageTypeMap.put(Quat4f.class, "geometry_msgs/Quaternion");

      javaClassToRosMessageTypeMap.put(Point3d.class, "geometry_msgs/Vector3");
      javaClassToRosMessageTypeMap.put(Point3f.class, "geometry_msgs/Vector3");
      javaClassToRosMessageTypeMap.put(Vector3d.class, "geometry_msgs/Vector3");
      javaClassToRosMessageTypeMap.put(Vector3f.class, "geometry_msgs/Vector3");
   }

   @SuppressWarnings("unchecked")
   public static <T extends Enum> String getDocumentation(Class<? extends T> documentedEnumClass, T documentedEnum)
   {
      String failureMessage = "Cannot get documentation for " + documentedEnum.toString() + " of Enum type " + documentedEnumClass.getCanonicalName() + "."
            + "Enum fields must either have the @RosEnumValueDocumentation annotation, or if they are low level must implement the method getDocumentation()."
            + "For this particular Enum type, you should implement the following method in " + documentedEnumClass.getCanonicalName() + ":\n\n"
            + "\tpublic static String getDocumentation(" + documentedEnumClass.getSimpleName() + " documentedValue)";
      try
      {
         Set<Method> reflectionUtilsResult = ReflectionUtils
               .getMethods(documentedEnumClass, ReflectionUtils.withModifier(Modifier.PUBLIC), ReflectionUtils.withModifier(Modifier.STATIC),
                     ReflectionUtils.withName("getDocumentation"));

         if(reflectionUtilsResult.isEmpty())
         {
            throw new RuntimeException(failureMessage);
         }

         for (Method method : reflectionUtilsResult)
         {
            return (String) method.invoke(null, documentedEnum);
         }
      }
      catch(Exception e)
      {
         throw new RuntimeException(failureMessage, e);
      }

      return null;
   }
}
