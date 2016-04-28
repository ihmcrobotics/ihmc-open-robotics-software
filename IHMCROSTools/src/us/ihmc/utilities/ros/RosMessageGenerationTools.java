package us.ihmc.utilities.ros;

import org.apache.commons.lang3.StringUtils;
import org.reflections.ReflectionUtils;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.tools.io.printing.PrintTools;

import javax.vecmath.*;
import java.lang.reflect.*;
import java.util.Collection;
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

      javaClassToRosMessageTypeMap.put(Point2d.class, "ihmc_msgs/Point2dRosMessage");

      javaClassToRosMessageTypeMap.put(Quat4d.class, "geometry_msgs/Quaternion");
      javaClassToRosMessageTypeMap.put(Quat4f.class, "geometry_msgs/Quaternion");

      javaClassToRosMessageTypeMap.put(Point3d.class, "geometry_msgs/Vector3");
      javaClassToRosMessageTypeMap.put(Point3f.class, "geometry_msgs/Vector3");
      javaClassToRosMessageTypeMap.put(Vector3d.class, "geometry_msgs/Vector3");
      javaClassToRosMessageTypeMap.put(Vector3f.class, "geometry_msgs/Vector3");
   }

   public static String getRosTypeForJavaType(Field field, Class<?> javaType)
   {
      if (javaClassToRosMessageTypeMap.containsKey(javaType))
      {
         return javaClassToRosMessageTypeMap.get(javaType);
      }
      else if (javaType.isArray() && javaClassToRosMessageTypeMap.containsKey(javaType.getComponentType()))
      {
         return javaClassToRosMessageTypeMap.get(javaType.getComponentType()) + "[]";
      }
      else if(Enum.class.isAssignableFrom(javaType))
      {
         return "uint8";
      }
      else if(Collection.class.isAssignableFrom(javaType))
      {
         ParameterizedType fieldGenericType = (ParameterizedType) field.getGenericType();

         return getRosTypeForJavaType(field, (Class<?>) fieldGenericType.getActualTypeArguments()[0]) + "[]";
      }
      else
      {
         boolean isArray = javaType.isArray();
         Class<?> workingClass = isArray ? javaType.getComponentType() : javaType;

         if(workingClass.isAnnotationPresent(RosMessagePacket.class))
         {
            RosMessagePacket annotation = workingClass.getAnnotation(RosMessagePacket.class);
            String messageName = workingClass.getSimpleName();

            if(!messageName.endsWith("Message"))
            {
               messageName += "Message";
            }

            messageName = StringUtils.replace(messageName, "Message", "RosMessage");

            String retString = annotation.rosPackage() + "/" + messageName;
            if(isArray)
            {
               retString += "[]";
            }

            return retString;
         }
         else
         {
            System.out.println("wat");
            System.out.println(javaType.getSimpleName());
            System.out.println(field.getDeclaringClass().getSimpleName());
            System.out.println();
         }
      }

      return null;
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

         if (reflectionUtilsResult.isEmpty())
         {
            throw new RuntimeException(failureMessage);
         }

         for (Method method : reflectionUtilsResult)
         {
            return (String) method.invoke(null, documentedEnum);
         }
      }
      catch (Exception e)
      {
         PrintTools.error(failureMessage);
      }

      return null;
   }

   @SuppressWarnings("unchecked")
   public static <T extends Enum> boolean hasDocumentation(Class<? extends T> documentedEnumClass)
   {
      return !ReflectionUtils.getMethods(documentedEnumClass, ReflectionUtils.withModifier(Modifier.PUBLIC), ReflectionUtils.withModifier(Modifier.STATIC),
            ReflectionUtils.withName("getDocumentation")).isEmpty();
   }

   public static String getRosMessageClassNameFromIHMCMessage(String messageName)
   {
      if(!messageName.endsWith("Message"))
      {
          messageName += "Message";
      }

      messageName = StringUtils.replace(messageName, "Message", "RosMessage");
      return messageName;
   }
}
