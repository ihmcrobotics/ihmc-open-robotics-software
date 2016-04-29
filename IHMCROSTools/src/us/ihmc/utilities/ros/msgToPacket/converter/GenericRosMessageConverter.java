package us.ihmc.utilities.ros.msgToPacket.converter;

import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import ihmc_msgs.Point2dRosMessage;
import org.apache.commons.lang3.StringUtils;
import org.reflections.ReflectionUtils;
import org.reflections.Reflections;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.utilities.ros.RosMessageGenerationTools;

import javax.vecmath.*;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.*;

public class GenericRosMessageConverter
{
   private static final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

   private static final Reflections ihmcPackageReflector = new Reflections("us.ihmc.humanoidRobotics.communication.packets");
   private static Set<Class<?>> rosMessagePacketAnnotatedTypes = null;

   public static MessageFactory getMessageFactory()
   {
      return messageFactory;
   }

   public static Reflections getIhmcPackageReflector()
   {
      return ihmcPackageReflector;
   }

   public static Message convertIHMCMessageToRosMessage(Packet<?> ihmcMessage)
         throws IllegalAccessException, ClassNotFoundException, NoSuchMethodException, InvocationTargetException
   {
      Class<? extends Packet> ihmcMessageClass = ihmcMessage.getClass();
      String rosMessageClassNameFromIHMCMessage = RosMessageGenerationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      Message message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      ArrayList<Field> fields = new ArrayList<>();
      for (Field field : ihmcMessageClass.getFields())
      {
         if (field.isAnnotationPresent(RosExportedField.class))
         {
            fields.add(field);
         }
      }

      convertIHMCMessageFieldsToROSFields(ihmcMessage, message, fields);

      return message;
   }

   @SuppressWarnings("unchecked")
   public static Packet<?> convertRosMessageToIHMCMessage(Message rosMessage)
         throws ClassNotFoundException, NoSuchFieldException, IllegalAccessException, InstantiationException, InvocationTargetException,
         RosEnumConversionException
   {
      Set<Class<?>> typesAnnotatedWith = getRosMessagePacketAnnotatedClasses();
      String fullRosTypeName = rosMessage.toRawMessage().getType();
      String rosMessageName = fullRosTypeName.split("/")[1];
      Class<?> rosMessageClass = Class.forName(fullRosTypeName.replace("/", "."));

      Class<? extends Packet> ihmcMessageClass = getIHMCMessageClassForROSMessage(typesAnnotatedWith, rosMessageName);

      Map<Method, Field> rosGetterToIHMCFieldMap = new HashMap<>();

      for (Method getter : ReflectionUtils.getMethods(rosMessageClass, ReflectionUtils.withPrefix("get")))
      {
         String fieldName = StringUtils.uncapitalize(getter.getName().replace("get", ""));
         try
         {
            Field field = ihmcMessageClass.getField(fieldName);
            rosGetterToIHMCFieldMap.put(getter, field);
         }
         catch(NoSuchFieldException e)
         {
            System.out.println("Couldn't find field " + fieldName + " for class " + ihmcMessageClass.getSimpleName());
         }
      }

      if(!rosGetterToIHMCFieldMap.isEmpty())
      {
         Packet<?> ihmcMessage = ihmcMessageClass.newInstance();
         for (Map.Entry<Method, Field> methodFieldEntry : rosGetterToIHMCFieldMap.entrySet())
         {
            Method rosGetter = methodFieldEntry.getKey();
            Field ihmcField = methodFieldEntry.getValue();

            Class<?> ihmcMessageFieldType = ihmcField.getType();
            if(List.class.isAssignableFrom(rosGetter.getReturnType()) && ihmcMessageFieldType.isArray())
            {
               setArrayFromList(rosMessage, ihmcMessage, rosGetter, ihmcField, ihmcMessageFieldType);
            }
            else if(ihmcMessageFieldType.isEnum())
            {
               setEnumFromByte(rosMessage, ihmcMessage, rosGetter, ihmcField, (Class<? extends Enum>) ihmcMessageFieldType);
            }
            else if(ihmcMessageFieldType.getCanonicalName().contains("javax.vecmath"))
            {
               setVecmathFieldFromRosGeometryMessage(rosMessage, ihmcMessage, rosGetter, ihmcField, ihmcMessageFieldType);
            }
            else
            {
               ihmcField.set(ihmcMessage, rosGetter.invoke(rosMessage));
            }
         }

         return ihmcMessage;
      }

      return null;
   }

   private static void setVecmathFieldFromRosGeometryMessage(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField,
         Class<?> ihmcMessageFieldType) throws IllegalAccessException, InvocationTargetException, InstantiationException
   {
      if(ihmcMessageFieldType.equals(Point2d.class))
      {
         Point2d point2d = convertPoint2DRos((Point2dRosMessage) rosGetter.invoke(rosMessage));
         ihmcField.set(ihmcMessage, point2d);
      }
      else
      {
         if(rosGetter.getReturnType().equals(Quaternion.class))
         {
            Tuple4d newTuple = (Tuple4d) ihmcField.getType().newInstance();
            newTuple.set(convertQuaternion((Quaternion) rosGetter.invoke(rosMessage)));
            ihmcField.set(ihmcMessage, newTuple);
         }
         else if(rosGetter.getReturnType().equals(Vector3.class))
         {
            Tuple3d newTuple = (Tuple3d) ihmcField.getType().newInstance();
            newTuple.set(convertVector3((Vector3) rosGetter.invoke(rosMessage)));
            ihmcField.set(ihmcMessage, newTuple);
         }
      }
   }

   private static void setEnumFromByte(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField, Class<? extends Enum> fieldType)
         throws IllegalAccessException, InvocationTargetException, RosEnumConversionException
   {
      Class<? extends Enum> enumClass = fieldType;
      byte ordinal = (byte) rosGetter.invoke(rosMessage);

      Enum[] enumConstants = enumClass.getEnumConstants();
      if(ordinal >= enumConstants.length)
      {
         throw new RosEnumConversionException(enumClass, ordinal, "");
      }
      else
      {
         ihmcField.set(ihmcMessage, enumConstants[ordinal]);
      }
   }

   private static void setArrayFromList(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField, Class<?> fieldType)
         throws IllegalAccessException, InvocationTargetException
   {
      List rosValues = (List) rosGetter.invoke(rosMessage);

      Class<?> componentType = fieldType.getComponentType();
      Object ihmcArray = Array.newInstance(componentType, rosValues.size());

      int i = 0;
      for(Object value : rosValues)
      {
         Array.set(ihmcArray, i, value);
         i++;
      }

      ihmcField.set(ihmcMessage, ihmcArray);
   }

   private static Class<? extends Packet> getIHMCMessageClassForROSMessage(Set<Class<?>> typesAnnotatedWith, String rosMessageName)
   {
      String ihmcMessageClassName = rosMessageName.replace("RosMessage", "Message");
      if(ihmcMessageClassName.endsWith("PacketMessage"))
      {
         ihmcMessageClassName = ihmcMessageClassName.replace("PacketMessage", "Packet");
      }

      Class<? extends Packet> ihmcMessageClass = null;
      for (Class<?> aClass : typesAnnotatedWith)
      {

         if(aClass.getSimpleName().equals(ihmcMessageClassName))
         {
            ihmcMessageClass = (Class<? extends Packet>) aClass;
            break;
         }
         else if(StatusPacket.class.isAssignableFrom(aClass) && aClass.getSimpleName().equals(ihmcMessageClassName.replace("Message", "")))
         {
            ihmcMessageClass = (Class<? extends Packet>) aClass;
            break;
         }
      }
      return ihmcMessageClass;
   }

   private static Set<Class<?>> getRosMessagePacketAnnotatedClasses()
   {
      if(rosMessagePacketAnnotatedTypes == null)
      {
         rosMessagePacketAnnotatedTypes = ihmcPackageReflector.getTypesAnnotatedWith(RosMessagePacket.class);
      }
      return rosMessagePacketAnnotatedTypes;
   }

   private static void convertIHMCMessageFieldsToROSFields(Packet<?> ihmcMessage, Message message, ArrayList<Field> fields)
         throws IllegalAccessException, NoSuchMethodException, InvocationTargetException, ClassNotFoundException
   {
      for (Field field : fields)
      {
         if (field.getType().getCanonicalName().contains("javax.vecmath"))
         {
            convertVecmathField(ihmcMessage, message, field);
         }
         else if (field.getType().isArray())
         {
            setListFromArray(ihmcMessage, message, field);
         }
         else if (Enum.class.isAssignableFrom(field.getType()))
         {
            setByteFromEnum(ihmcMessage, message, field);
         }
         else
         {
            setField(message, field, field.get(ihmcMessage));
         }
      }
   }

   private static void convertVecmathField(Packet<?> ihmcMessage, Message message, Field field)
         throws IllegalAccessException, NoSuchMethodException, InvocationTargetException, ClassNotFoundException
   {
      if (field.getType().equals(Point2d.class))
      {
         Point2d point = (Point2d) field.get(ihmcMessage);
         setPoint2dField(message, field, point);
      }
      else
      {
         Object vecmathObject = field.get(ihmcMessage);
         setVecmathField(message, field, vecmathObject);
      }
   }

   private static void setPoint2dField(Message message, Field field, Point2d value)
         throws NoSuchMethodException, InvocationTargetException, IllegalAccessException
   {
      String setterName = getRosGetterNameForField(field);
      Method setterMethod = message.getClass().getMethod(setterName, Point2dRosMessage.class);
      setterMethod.setAccessible(true);

      setterMethod.invoke(message, convertPoint2d(value));
   }

   private static void setVecmathField(Message message, Field field, Object value)
         throws ClassNotFoundException, NoSuchMethodException, InvocationTargetException, IllegalAccessException
   {
      String setterName = getRosSetterNameForField(field);
      String rosTypeForJavaType = RosMessageGenerationTools.getRosTypeForJavaType(field, field.getType());

      assert rosTypeForJavaType != null;
      Method setterMethod = message.getClass().getMethod(setterName, Class.forName(rosTypeForJavaType.replace("/", ".")));
      setterMethod.setAccessible(true);

      Class<?> vecmathClass = field.getType();
      Class<?> genericVecmathClass = Class.forName(vecmathClass.getGenericSuperclass().getTypeName());
      String genericVecmathClassName = genericVecmathClass.getSimpleName();
      Method converterMethod = GenericRosMessageConverter.class.getMethod("convert" + genericVecmathClassName, genericVecmathClass);
      converterMethod.setAccessible(true);

      setterMethod.invoke(message, converterMethod.invoke(null, value));
   }

   private static void setField(Message message, Field field, Object value) throws NoSuchMethodException, InvocationTargetException, IllegalAccessException
   {
      Method rosSetterForField = getRosSetterForField(message.getClass(), field);
      rosSetterForField.invoke(message, value);
   }

   private static void setByteFromEnum(Packet<?> ihmcMessage, Message message, Field field)
         throws NoSuchMethodException, InvocationTargetException, IllegalAccessException
   {
      Method rosSetterForField = getRosSetterForField(message.getClass(), field);
      Enum enumField = (Enum) field.get(ihmcMessage);
      if(enumField != null)
      {
         rosSetterForField.invoke(message, (byte) enumField.ordinal());
      }
   }

   private static void setListFromArray(Packet<?> ihmcMessage, Message message, Field field)
         throws NoSuchMethodException, InvocationTargetException, IllegalAccessException
   {
      Object[] fieldAsArray = (Object[]) field.get(ihmcMessage);

      String setterMethodName = getRosSetterNameForField(field);
      Method listSetterMethod;

      if(field.getType().isArray() && field.getType().getComponentType().isPrimitive())
      {
         listSetterMethod = message.getClass().getMethod(setterMethodName, field.getType());
         listSetterMethod.setAccessible(true);
         if (fieldAsArray == null)
         {
            listSetterMethod.invoke(message, Array.newInstance(field.getType().getComponentType(), 0));
         }
         else
         {
            listSetterMethod.invoke(message, fieldAsArray);
         }
      }
      else
      {
         List<Object> objects;

         if (fieldAsArray == null)
         {
            objects = new ArrayList<>();
         }
         else
         {
            objects = Arrays.asList(fieldAsArray);
         }

         listSetterMethod = message.getClass().getMethod(setterMethodName, List.class);
         listSetterMethod.setAccessible(true);
         listSetterMethod.invoke(message, objects);
      }

   }

   private static Method getRosGetterForField(Class<? extends Message> rosMessageClass, Field field) throws NoSuchMethodException
   {
      String methodName = getRosGetterNameForField(field);
      Method method = rosMessageClass.getMethod(methodName);
      method.setAccessible(true);

      return method;
   }

   private static Method getRosSetterForField(Class<? extends Message> rosMessageClass, Field field) throws NoSuchMethodException
   {
      String methodName = getRosSetterNameForField(field);
      Class<?> type = field.getType().isEnum() ? byte.class : field.getType();

      Method method = rosMessageClass.getMethod(methodName, type);
      method.setAccessible(true);

      return method;
   }

   private static String getRosGetterNameForField(Field field)
   {
      return "get" + StringUtils.capitalize(field.getName());
   }

   private static String getRosSetterNameForField(Field field)
   {
      return "set" + StringUtils.capitalize(field.getName());
   }

   public static Point2d convertPoint2DRos(Point2dRosMessage point2dRosMessage)
   {
      if(point2dRosMessage == null)
         return new Point2d(Double.NaN, Double.NaN);

      Point2d point = new Point2d(point2dRosMessage.getX(), point2dRosMessage.getY());

      return point;
   }

   public static Tuple3d convertVector3(Vector3 vector3)
   {
      if(vector3 == null)
         return new Point3d(Double.NaN, Double.NaN, Double.NaN);

      Point3d point = new Point3d(vector3.getX(), vector3.getY(), vector3.getZ());

      return point;
   }

   public static Tuple4d convertQuaternion(Quaternion quaternion)
   {
      if(quaternion == null)
         return null;
//         return new Quat4d(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

      Quat4d quat = new Quat4d(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());

      return quat;
   }

   /*
    * Do not delete, used by reflection!
    */
   public static Vector3 convertTuple3d(Tuple3d tuple)
   {
      Vector3 vector3 = messageFactory.newFromType("geometry_msgs/Vector3");
      if(tuple == null)
      {
         vector3.setX(Double.NaN);
         vector3.setY(Double.NaN);
         vector3.setZ(Double.NaN);
      }
      else
      {
         vector3.setX(tuple.getX());
         vector3.setY(tuple.getY());
         vector3.setZ(tuple.getZ());
      }

      return vector3;
   }

   /*
    * Do not delete, used by reflection!
    */
   public static Vector3 convertTuple3f(Tuple3f tuple)
   {
      Vector3 vector3 = messageFactory.newFromType("geometry_msgs/Vector3");

      if(tuple == null)
      {
         vector3.setX(Double.NaN);
         vector3.setY(Double.NaN);
         vector3.setZ(Double.NaN);
      }
      else
      {
         vector3.setX(tuple.getX());
         vector3.setY(tuple.getY());
         vector3.setZ(tuple.getZ());
      }

      return vector3;
   }

   /*
    * Do not delete, used by reflection!
    */
   public static Quaternion convertTuple4d(Tuple4d tuple)
   {
      Quaternion quaternion = messageFactory.newFromType("geometry_msgs/Quaternion");
      if(tuple == null)
      {
         quaternion.setX(Double.NaN);
         quaternion.setY(Double.NaN);
         quaternion.setZ(Double.NaN);
         quaternion.setW(Double.NaN);
      }
      else
      {
         quaternion.setX(tuple.getX());
         quaternion.setY(tuple.getY());
         quaternion.setZ(tuple.getZ());
         quaternion.setW(tuple.getW());
      }

      return quaternion;
   }

   /*
    * Do not delete, used by reflection!
    */
   public static Quaternion convertTuple4f(Tuple4f tuple)
   {
      Quaternion quaternion = messageFactory.newFromType("geometry_msgs/Quaternion");
      if(tuple == null)
      {
         quaternion.setX(Double.NaN);
         quaternion.setY(Double.NaN);
         quaternion.setZ(Double.NaN);
         quaternion.setW(Double.NaN);
      }
      else
      {
         quaternion.setX(tuple.getX());
         quaternion.setY(tuple.getY());
         quaternion.setZ(tuple.getZ());
         quaternion.setW(tuple.getW());
      }

      return quaternion;
   }

   public static Point2dRosMessage convertPoint2d(Point2d point2d)
   {
      Point2dRosMessage point2dMessage = messageFactory.newFromType("ihmc_msgs/Point2dRosMessage");
      if(point2d == null)
      {
         point2dMessage.setX(Double.NaN);
         point2dMessage.setY(Double.NaN);
      }
      else
      {
         point2dMessage.setX(point2d.getX());
         point2dMessage.setY(point2d.getY());
      }

      return point2dMessage;
   }
}
