package us.ihmc.utilities.ros.msgToPacket.converter;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.ParameterizedType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.commons.lang3.StringUtils;
import org.reflections.ReflectionUtils;
import org.reflections.Reflections;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import geometry_msgs.Point;
import geometry_msgs.Transform;
import geometry_msgs.Vector3;
import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TLongArrayList;
import ihmc_msgs.Point2dRosMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.idl.PreallocatedList;

public class GenericROSTranslationTools
{
   private static final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

   private static final Reflections ihmcPackageReflector = new Reflections("us.ihmc.");
   private static Set<Class<?>> ihmcCoreAnnotatedPacket = null;
   private static Set<Class<?>> allAnnotatedPackets = null;
   private static Set<Class<?>> outputTopics;
   private static Set<Class<?>> inputTopics;

   private static final HashMap<Class<?>, String> javaClassToRosMessageTypeMap = new HashMap<>();
   private static CustomFieldConversions customFieldConversions = CustomFieldConversions.getInstance();

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

      javaClassToRosMessageTypeMap.put(Point2D.class, "ihmc_msgs/Point2dRosMessage");

      javaClassToRosMessageTypeMap.put(Quaternion.class, "geometry_msgs/Quaternion");
      javaClassToRosMessageTypeMap.put(Quaternion32.class, "geometry_msgs/Quaternion");

      javaClassToRosMessageTypeMap.put(Point3D.class, "geometry_msgs/Point");
      javaClassToRosMessageTypeMap.put(Point3D32.class, "geometry_msgs/Point");
      javaClassToRosMessageTypeMap.put(Vector3D.class, "geometry_msgs/Vector3");

      javaClassToRosMessageTypeMap.put(Pose3D.class, "geometry_msgs/Transform");

      // QuaternionBasedTransform <-> Transform
      customFieldConversions.registerIHMCPacketFieldConverter(Pose3D.class, GenericROSTranslationTools::convertPose3DToTransform);
      customFieldConversions.registerROSMessageFieldConverter(Transform.class, GenericROSTranslationTools::convertTransformToPose3D);

      // Point3D <-> Point
      customFieldConversions.registerIHMCPacketFieldConverter(Point3D.class, GenericROSTranslationTools::convertPoint3D);
      customFieldConversions.registerROSMessageFieldConverter(Point.class, GenericROSTranslationTools::convertPoint);

      // Vector3D <-> Vector3
      customFieldConversions.registerIHMCPacketFieldConverter(Vector3D.class, GenericROSTranslationTools::convertVector3D);
      customFieldConversions.registerROSMessageFieldConverter(Vector3.class, GenericROSTranslationTools::convertVector3);

      // Quaternion <-> ROS Quaternion
      customFieldConversions.registerIHMCPacketFieldConverter(Quaternion.class, GenericROSTranslationTools::convertTuple4d);
      customFieldConversions.registerROSMessageFieldConverter(geometry_msgs.Quaternion.class, GenericROSTranslationTools::convertQuaternion);
   }

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
      return convertJavaObjectToRosMessage(ihmcMessage);
   }

   private static Message convertJavaObjectToRosMessage(Object ihmcObject)
         throws InvocationTargetException, NoSuchMethodException, ClassNotFoundException, IllegalAccessException
   {
      if (!ihmcObject.getClass().isAnnotationPresent(RosMessagePacket.class))
         throw new IllegalArgumentException("Class " + ihmcObject.getClass().getSimpleName()
               + " must contain RosMessagePacket class annotation to be converted.");

      Class<?> ihmcMessageClass = ihmcObject.getClass();
      String rosMessageClassNameFromIHMCMessage = getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
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

      convertIHMCMessageFieldsToROSFields(ihmcObject, message, fields);

      return message;
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static Packet<?> convertRosMessageToIHMCMessage(Message rosMessage) throws ClassNotFoundException, NoSuchFieldException, IllegalAccessException,
         InstantiationException, InvocationTargetException, RosEnumConversionException, IllegalArgumentException, NoSuchMethodException, SecurityException
   {
      Set<Class<?>> typesAnnotatedWith = getAllRosMessagePacketAnnotatedClasses();
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
         catch (NoSuchFieldException e)
         {
            System.out.println("Couldn't find field " + fieldName + " for class " + ihmcMessageClass.getSimpleName());
         }
      }

      if (!rosGetterToIHMCFieldMap.isEmpty())
      {
         Packet<?> ihmcMessage = ihmcMessageClass.newInstance();
         for (Map.Entry<Method, Field> methodFieldEntry : rosGetterToIHMCFieldMap.entrySet())
         {
            Method rosGetter = methodFieldEntry.getKey();
            Field ihmcField = methodFieldEntry.getValue();

            Class<?> ihmcMessageFieldType = ihmcField.getType();
            if (List.class.isAssignableFrom(rosGetter.getReturnType()) && ihmcMessageFieldType.isArray())
            {
               setArrayFromList(rosMessage, ihmcMessage, rosGetter, ihmcField, ihmcMessageFieldType);
            }
            else if (List.class.isAssignableFrom(rosGetter.getReturnType()) && ihmcMessageFieldType == PreallocatedList.class)
            {
               setPreallocatedListFromList(rosMessage, ihmcMessage, rosGetter, ihmcField, ihmcMessageFieldType);
            }
            else if (double[].class.isAssignableFrom(rosGetter.getReturnType()) && ihmcMessageFieldType == TDoubleArrayList.class)
            {
               setTDoubleArrayListFromArray(rosMessage, ihmcMessage, rosGetter, ihmcField, ihmcMessageFieldType);
            }
            else if (float[].class.isAssignableFrom(rosGetter.getReturnType()) && ihmcMessageFieldType == TFloatArrayList.class)
            {
               setTFloatArrayListFromArray(rosMessage, ihmcMessage, rosGetter, ihmcField, ihmcMessageFieldType);
            }
            else if (int[].class.isAssignableFrom(rosGetter.getReturnType()) && ihmcMessageFieldType == TIntArrayList.class)
            {
               setTIntArrayListFromArray(rosMessage, ihmcMessage, rosGetter, ihmcField, ihmcMessageFieldType);
            }
            else if (byte[].class.isAssignableFrom(rosGetter.getReturnType()) && ihmcMessageFieldType == TByteArrayList.class)
            {
               setTByteArrayListFromArray(rosMessage, ihmcMessage, rosGetter, ihmcField, ihmcMessageFieldType);
            }
            else if (ihmcMessageFieldType.isEnum())
            {
               setEnumFromByte(rosMessage, ihmcMessage, rosGetter, ihmcField, (Class<? extends Enum>) ihmcMessageFieldType);
            }
            else if (customFieldConversions.containsConverterFor(rosGetter.getReturnType()))
            {
               Message rosMessageField = (Message) rosGetter.invoke(rosMessage);
               Object ihmcPacketField = customFieldConversions.convert(rosMessageField);
               ihmcField.set(ihmcMessage, ihmcPacketField);
            }
            else
            {
               Object rosField = rosGetter.invoke(rosMessage);
               if (rosField instanceof Message)
                  rosField = convertRosMessageToIHMCMessage((Message) rosField);

               ihmcField.set(ihmcMessage, rosField);
            }
         }

         return ihmcMessage;
      }

      return null;
   }

   @SuppressWarnings("rawtypes")
   private static void setEnumFromByte(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField, Class<? extends Enum> fieldType)
         throws IllegalAccessException, InvocationTargetException, RosEnumConversionException
   {
      Class<? extends Enum> enumClass = fieldType;
      byte ordinal = (byte) rosGetter.invoke(rosMessage);

      Enum[] enumConstants = enumClass.getEnumConstants();
      if (ordinal >= enumConstants.length)
      {
         throw new RosEnumConversionException(enumClass, ordinal, "");
      }
      else
      {
         ihmcField.set(ihmcMessage, enumConstants[ordinal]);
      }
   }

   private static void setArrayFromList(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField, Class<?> fieldType)
         throws IllegalAccessException, InvocationTargetException, ClassNotFoundException, InstantiationException, RosEnumConversionException,
         NoSuchFieldException, ArrayIndexOutOfBoundsException, IllegalArgumentException, NoSuchMethodException, SecurityException
   {
      List<?> rosValues = (List<?>) rosGetter.invoke(rosMessage);

      Class<?> componentType = fieldType.getComponentType();
      Object ihmcArray = Array.newInstance(componentType, rosValues.size());

      int i = 0;
      for (Object value : rosValues)
      {
         if (value instanceof Message)
         {
            Array.set(ihmcArray, i, convertRosMessageToIHMCMessage((Message) value));
         }
         else
         {
            Array.set(ihmcArray, i, value);
         }

         i++;
      }

      ihmcField.set(ihmcMessage, ihmcArray);
   }

   private static void setTDoubleArrayListFromArray(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField, Class<?> fieldType)
         throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      double[] rosValues = (double[]) rosGetter.invoke(rosMessage);
      ihmcField.set(ihmcMessage, new TDoubleArrayList(rosValues));
   }

   private static void setTFloatArrayListFromArray(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField, Class<?> fieldType)
         throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      float[] rosValues = (float[]) rosGetter.invoke(rosMessage);
      ihmcField.set(ihmcMessage, new TFloatArrayList(rosValues));
   }

   private static void setTIntArrayListFromArray(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField, Class<?> fieldType)
         throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      int[] rosValues = (int[]) rosGetter.invoke(rosMessage);
      ihmcField.set(ihmcMessage, new TIntArrayList(rosValues));
   }

   private static void setTByteArrayListFromArray(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField, Class<?> fieldType)
         throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      byte[] rosValues = (byte[]) rosGetter.invoke(rosMessage);
      ihmcField.set(ihmcMessage, new TByteArrayList(rosValues));
   }

   @SuppressWarnings({"rawtypes"})
   private static void setPreallocatedListFromList(Message rosMessage, Packet<?> ihmcMessage, Method rosGetter, Field ihmcField, Class<?> fieldType)
         throws IllegalAccessException, InvocationTargetException, ClassNotFoundException, InstantiationException, RosEnumConversionException,
         NoSuchFieldException, IllegalArgumentException, NoSuchMethodException, SecurityException
   {
      List<?> rosValues = (List<?>) rosGetter.invoke(rosMessage);

      PreallocatedList<?> ihmcList = (PreallocatedList<?>) ihmcField.get(ihmcMessage);

      for (Object rosValue : rosValues)
      {
         Object ihmcValue;
         if (rosValue instanceof Message)
         {
            ihmcValue = convertRosMessageToIHMCMessage((Message) rosValue);
         }
         else
         {
            ihmcValue = rosValue;
         }

         Object ihmcSettable = ihmcList.add();
         ihmcSettable.getClass().getMethod("set", ihmcValue.getClass()).invoke(ihmcSettable, ihmcValue);
      }

      ihmcField.set(ihmcMessage, ihmcList);
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   public static Class<? extends Packet> getIHMCMessageClassForROSMessage(Set<Class<?>> typesAnnotatedWith, String rosMessageName)
   {
      String ihmcMessageClassName = rosMessageName.replace("RosMessage", "Message");
      if (ihmcMessageClassName.endsWith("PacketMessage"))
      {
         ihmcMessageClassName = ihmcMessageClassName.replace("PacketMessage", "Packet");
      }

      Class<? extends Packet> ihmcMessageClass = null;
      for (Class<?> aClass : typesAnnotatedWith)
      {

         if (aClass.getSimpleName().equals(ihmcMessageClassName))
         {
            ihmcMessageClass = (Class<? extends Packet>) aClass;
            break;
         }
         else if (aClass.getSimpleName().equals(ihmcMessageClassName.replace("Message", "")))
         {
            ihmcMessageClass = (Class<? extends Packet>) aClass;
            break;
         }
      }
      return ihmcMessageClass;
   }

   public static Set<Class<?>> getAllRosMessagePacketAnnotatedClasses()
   {
      if (allAnnotatedPackets == null)
      {
         allAnnotatedPackets = ihmcPackageReflector.getTypesAnnotatedWith(RosMessagePacket.class);
      }

      return allAnnotatedPackets;
   }

   public static Set<Class<?>> getIHMCCoreRosMessagePacketAnnotatedClasses()
   {
      if (ihmcCoreAnnotatedPacket == null)
      {
         ihmcCoreAnnotatedPacket = new HashSet<>();

         for (Class<?> aClass : getAllRosMessagePacketAnnotatedClasses())
         {
            RosMessagePacket annotation = aClass.getAnnotation(RosMessagePacket.class);
            if (annotation.rosPackage().equals(RosMessagePacket.CORE_IHMC_PACKAGE))
            {
               ihmcCoreAnnotatedPacket.add(aClass);
            }
         }
      }

      return ihmcCoreAnnotatedPacket;
   }

   public static Set<Class<?>> getCoreOutputTopics()
   {
      if (outputTopics == null)
      {
         outputTopics = new HashSet<>();
         for (Class<?> aClass : getIHMCCoreRosMessagePacketAnnotatedClasses())
         {
            if (!aClass.getAnnotation(RosMessagePacket.class).topic().equals(RosMessagePacket.NO_CORRESPONDING_TOPIC_STRING))
            {
               outputTopics.add(aClass);
            }
         }
      }

      return outputTopics;
   }

   public static Set<Class<?>> getCoreInputTopics()
   {
      if (inputTopics == null)
      {
         inputTopics = new HashSet<>();
         for (Class<?> aClass : getIHMCCoreRosMessagePacketAnnotatedClasses())
         {
            if (!aClass.getAnnotation(RosMessagePacket.class).topic().equals(RosMessagePacket.NO_CORRESPONDING_TOPIC_STRING))
            {
               inputTopics.add(aClass);
            }
         }
      }

      return inputTopics;
   }

   public static Set<Class<?>> getOutputTopicsForPackage(String additionalPackage)
   {
      Set<Class<?>> allAnnotatedRosMessageClasses = getAllRosMessagePacketAnnotatedClasses();
      Set<Class<?>> outputTopicsForPackage = new HashSet<>();

      for (Class<?> annotatedClass : allAnnotatedRosMessageClasses)
      {
         RosMessagePacket annotation = annotatedClass.getAnnotation(RosMessagePacket.class);
         if (annotation.rosPackage().equals(additionalPackage) && !annotation.topic().equals(RosMessagePacket.NO_CORRESPONDING_TOPIC_STRING))
         {
            outputTopicsForPackage.add(annotatedClass);
         }
      }
      return outputTopicsForPackage;
   }

   public static Set<Class<?>> getInputTopicsForPackage(String additionalPackage)
   {
      Set<Class<?>> allAnnotatedRosMessageClasses = getAllRosMessagePacketAnnotatedClasses();
      Set<Class<?>> inputTopicsForPackage = new HashSet<>();

      for (Class<?> annotatedClass : allAnnotatedRosMessageClasses)
      {
         RosMessagePacket annotation = annotatedClass.getAnnotation(RosMessagePacket.class);
         if (annotation.rosPackage().equals(additionalPackage) && !annotation.topic().equals(RosMessagePacket.NO_CORRESPONDING_TOPIC_STRING))
         {
            inputTopicsForPackage.add(annotatedClass);
         }
      }

      return inputTopicsForPackage;
   }

   private static void convertIHMCMessageFieldsToROSFields(Object ihmcMessage, Message message, ArrayList<Field> fields)
         throws IllegalAccessException, NoSuchMethodException, InvocationTargetException, ClassNotFoundException
   {
      for (Field field : fields)
      {
         if (field.getType().isAnnotationPresent(RosMessagePacket.class))
         {
            Message rosMessageField = convertJavaObjectToRosMessage(field.get(ihmcMessage));
            String rosSetterForField = getRosSetterNameForField(field);
            Method setterMethod = message.getClass().getMethod(rosSetterForField, rosMessageField.getClass().getInterfaces()[0]);
            setterMethod.setAccessible(true);
            setterMethod.invoke(message, rosMessageField);
         }
         else if (customFieldConversions.containsConverterFor(field.getType()))
         {
            Object fieldVariableToConvert = field.get(ihmcMessage);
            Message rosMessageField = customFieldConversions.convert(fieldVariableToConvert);

            String rosSetterForField = getRosSetterNameForField(field);
            Method setterMethod = message.getClass().getMethod(rosSetterForField, rosMessageField.getClass().getInterfaces()[0]);
            setterMethod.setAccessible(true);
            setterMethod.invoke(message, rosMessageField);
         }
         else if (field.getType().isArray() && !field.getType().getComponentType().isPrimitive())
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

   private static void setField(Message message, Field field, Object value) throws NoSuchMethodException, InvocationTargetException, IllegalAccessException
   {
      Method rosSetterForField = getRosSetterForField(message.getClass(), field);
      if (value instanceof PreallocatedList)
         value = Arrays.asList(((PreallocatedList<?>) value).toArray());
      else if (value instanceof TDoubleArrayList)
         value = ((TDoubleArrayList) value).toArray();
      else if (value instanceof TLongArrayList)
         value = ((TLongArrayList) value).toArray();
      else if (value instanceof TByteArrayList)
         value = ((TByteArrayList) value).toArray();
      else if (value instanceof TFloatArrayList)
         value = ((TFloatArrayList) value).toArray();
      else if (value instanceof TIntArrayList)
         value = ((TIntArrayList) value).toArray();
      rosSetterForField.invoke(message, value);
   }

   private static void setByteFromEnum(Object ihmcMessage, Message message, Field field)
         throws NoSuchMethodException, InvocationTargetException, IllegalAccessException
   {
      Method rosSetterForField = getRosSetterForField(message.getClass(), field);
      Enum<?> enumField = (Enum<?>) field.get(ihmcMessage);
      if (enumField != null)
      {
         rosSetterForField.invoke(message, (byte) enumField.ordinal());
      }
   }

   private static void setListFromArray(Object ihmcMessage, Message message, Field field)
         throws NoSuchMethodException, InvocationTargetException, IllegalAccessException
   {
      Object[] fieldAsArray = (Object[]) field.get(ihmcMessage);

      String setterMethodName = getRosSetterNameForField(field);
      Method listSetterMethod;

      if (field.getType().isArray() && field.getType().getComponentType().isPrimitive())
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

   private static Method getRosSetterForField(Class<? extends Message> rosMessageClass, Field field) throws NoSuchMethodException
   {
      String methodName = getRosSetterNameForField(field);
      Class<?> type = field.getType().isEnum() ? byte.class : field.getType();
      if (type == PreallocatedList.class)
         type = List.class;
      else if (type == TByteArrayList.class)
         type = byte[].class;
      else if (type == TLongArrayList.class)
         type = long[].class;
      else if (type == TDoubleArrayList.class)
         type = double[].class;
      else if (type == TFloatArrayList.class)
         type = float[].class;
      else if (type == TIntArrayList.class)
         type = int[].class;

      Method method = rosMessageClass.getMethod(methodName, type);
      method.setAccessible(true);

      return method;
   }

   private static String getRosSetterNameForField(Field field)
   {
      return "set" + StringUtils.capitalize(field.getName());
   }

   public static Point2D convertPoint2DRos(Point2dRosMessage point2dRosMessage)
   {
      if (point2dRosMessage == null)
         return new Point2D(Double.NaN, Double.NaN);

      Point2D point = new Point2D(point2dRosMessage.getX(), point2dRosMessage.getY());

      return point;
   }

   public static Vector3DBasics convertVector3(Vector3 vector3)
   {
      if (vector3 == null)
         return new Vector3D(Double.NaN, Double.NaN, Double.NaN);

      Vector3D vector = new Vector3D(vector3.getX(), vector3.getY(), vector3.getZ());

      return vector;
   }

   public static Point3DBasics convertPoint(Point point)
   {
      if (point == null)
         return new Point3D(Double.NaN, Double.NaN, Double.NaN);

      return new Point3D(point.getX(), point.getY(), point.getZ());
   }

   public static Tuple4DBasics convertQuaternion(geometry_msgs.Quaternion quaternion)
   {
      if (quaternion == null)
         return null;
      //         return new Quat4d(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

      Quaternion quat = new Quaternion(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());

      return quat;
   }

   /*
    * Do not delete, used by reflection!
    */
   public static Vector3 convertVector3D(Vector3DReadOnly tuple)
   {
      Vector3 vector3 = messageFactory.newFromType("geometry_msgs/Vector3");
      if (tuple == null)
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
   public static Point convertPoint3D(Point3DReadOnly point)
   {
      Point rosPoint = messageFactory.newFromType("geometry_msgs/Point");
      if (point == null)
      {
         rosPoint.setX(Double.NaN);
         rosPoint.setY(Double.NaN);
         rosPoint.setZ(Double.NaN);
      }
      else
      {
         rosPoint.setX(point.getX());
         rosPoint.setY(point.getY());
         rosPoint.setZ(point.getZ());
      }

      return rosPoint;
   }

   /*
    * Do not delete, used by reflection!
    */
   public static geometry_msgs.Quaternion convertTuple4d(Tuple4DReadOnly tuple)
   {
      geometry_msgs.Quaternion quaternion = messageFactory.newFromType("geometry_msgs/Quaternion");
      if (tuple == null)
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
         quaternion.setW(tuple.getS());
      }

      return quaternion;
   }

   public static Point2dRosMessage convertPoint2d(Point2D point2d)
   {
      Point2dRosMessage point2dMessage = messageFactory.newFromType("ihmc_msgs/Point2dRosMessage");
      if (point2d == null)
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
      else if (Enum.class.isAssignableFrom(javaType))
      {
         return "uint8";
      }
      else if (Collection.class.isAssignableFrom(javaType))
      {
         ParameterizedType fieldGenericType = (ParameterizedType) field.getGenericType();

         return getRosTypeForJavaType(field, (Class<?>) fieldGenericType.getActualTypeArguments()[0]) + "[]";
      }
      else
      {
         boolean isArray = javaType.isArray();
         Class<?> workingClass = isArray ? javaType.getComponentType() : javaType;

         if (workingClass.isAnnotationPresent(RosMessagePacket.class))
         {
            RosMessagePacket annotation = workingClass.getAnnotation(RosMessagePacket.class);
            String messageName = workingClass.getSimpleName();

            if (!messageName.endsWith("Message"))
            {
               messageName += "Message";
            }

            messageName = StringUtils.replace(messageName, "Message", "RosMessage");

            String retString = annotation.rosPackage() + "/" + messageName;
            if (isArray)
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
   public static <T extends Enum<?>> String getDocumentation(Class<? extends T> documentedEnumClass, T documentedEnum)
   {
      String failureMessage = "Cannot get documentation for " + documentedEnum.toString() + " of Enum type " + documentedEnumClass.getCanonicalName() + "."
            + "Enum fields must either have the @RosEnumValueDocumentation annotation, or if they are low level must implement the method getDocumentation()."
            + "For this particular Enum type, you should implement the following method in " + documentedEnumClass.getCanonicalName() + ":\n\n"
            + "\tpublic static String getDocumentation(" + documentedEnumClass.getSimpleName() + " documentedValue)";
      try
      {
         Set<Method> reflectionUtilsResult = ReflectionUtils.getMethods(documentedEnumClass, ReflectionUtils.withModifier(Modifier.PUBLIC),
                                                                        ReflectionUtils.withModifier(Modifier.STATIC),
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
   public static <T extends Enum<?>> boolean hasDocumentation(Class<? extends T> documentedEnumClass)
   {
      return !ReflectionUtils.getMethods(documentedEnumClass, ReflectionUtils.withModifier(Modifier.PUBLIC), ReflectionUtils.withModifier(Modifier.STATIC),
                                         ReflectionUtils.withName("getDocumentation"))
                             .isEmpty();
   }

   public static String getRosMessageClassNameFromIHMCMessage(String messageName)
   {
      if (!messageName.endsWith("Message"))
      {
         messageName += "Message";
      }

      messageName = StringUtils.replace(messageName, "Message", "RosMessage");
      return messageName;
   }

   private static Transform convertPose3DToTransform(Pose3D pose3D)
   {
      Transform message = messageFactory.newFromType("geometry_msgs/Transform");

      message.setTranslation(convertVector3D(new Vector3D(pose3D.getPosition())));
      message.setRotation(convertTuple4d(pose3D.getOrientation()));

      return message;
   }

   private static Pose3D convertTransformToPose3D(Transform transform)
   {
      Pose3D pose3D = new Pose3D();

      pose3D.setPosition(convertVector3(transform.getTranslation()));
      pose3D.setOrientation(new Quaternion(convertQuaternion(transform.getRotation())));

      return pose3D;
   }
}
