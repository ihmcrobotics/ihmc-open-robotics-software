package us.ihmc.utilities.ros.msgToPacket.converter;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.ParameterizedType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.apache.commons.lang3.ArrayUtils;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import com.google.common.base.CaseFormat;

/**
 * Created by agrabertilton on 4/23/15.
 */
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class GenericRosMessageConverter
{
   private static final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

   public static Message convertToIHMCRosMessage(Object packet)
           throws IllegalArgumentException, IllegalAccessException, InvocationTargetException, NoSuchMethodException, SecurityException, ClassNotFoundException
   {

      String simpleName = packet.getClass().getSimpleName();
      Field[] fields = packet.getClass().getFields();

      String messageClassName = "ihmc_msgs/" + simpleName + "Message";

      // create new instance of messageClassName
      Message message = messageFactory.newFromType(messageClassName);


      for (Field field : fields)
      {
         String fieldName = field.getName().replace("_", "");
         String methodName = getMethodNameForFieldName("set", fieldName);
         Class fieldType = field.getType();
         Method setValue = null;
         Object fieldValue = null;

         if (field.get(packet) == null)
         {
            continue;
         }

         if (isConstant(field) || isIgnoredField(field))
         {
            continue;
         }

         if (fieldType.isPrimitive() || fieldType.equals(String.class))
         {
            setValue = message.getClass().getMethod(methodName, fieldType);
            fieldValue = field.get(packet);
         }
         else if (fieldType.isEnum())
         {
            // will be going to uint8 field
            setValue = message.getClass().getMethod(methodName, byte.class);
            fieldValue = (byte) ((java.lang.Enum) field.get(packet)).ordinal();
         }
         else if (fieldType.isArray())
         {
            // will be in form field[]
            Class componentType = fieldType.getComponentType();
            if (componentType.equals(byte.class))
            {
               // Deal with ChannelBuffer set/get
               // must convert from byte[] to ChannelBuffer
               setValue = message.getClass().getMethod(methodName, ChannelBuffer.class);
               fieldValue = ChannelBuffers.wrappedBuffer((byte[]) field.get(packet));
            }
            else if (componentType.isEnum())
            {
               // is ENUM[]
               // will be going to uint8[] field, requires channelbuffer
               java.lang.Enum[] objects = ((java.lang.Enum[]) field.get(packet));
               byte[] values = new byte[objects.length];

               for (int i = 0; i < values.length; i++)
               {
                  values[i] = (byte) objects[i].ordinal();
               }

               setValue = message.getClass().getMethod(methodName, ChannelBuffer.class);
               fieldValue = ChannelBuffers.wrappedBuffer(values);
            }
            else if (componentType.isPrimitive())
            {
               // no conversion
               setValue = message.getClass().getMethod(methodName, fieldType);
               fieldValue = field.get(packet);
            }
            else if (componentType.equals(String.class))
            {
               setValue = message.getClass().getMethod(methodName, List.class);
               fieldValue = Arrays.asList(field.get(packet));
            }
            else if (componentType.equals(Point3d.class))
            {
               List<geometry_msgs.Vector3> value = new ArrayList<Vector3>();
               List<Point3d> points = Arrays.asList((Point3d[]) field.get(packet));

               for (Point3d point : points)
               {
                  value.add(convertPoint3dToVector3(point));
               }

               setValue = message.getClass().getMethod(methodName, value.getClass());
               fieldValue = value;
            }
            else if (componentType.equals(Vector3d.class))
            {
               List<geometry_msgs.Vector3> value = new ArrayList<geometry_msgs.Vector3>();
               List<Vector3d> vectors = Arrays.asList((Vector3d[]) field.get(packet));

               for (Vector3d vector3d : vectors)
               {
                  value.add(convertVector3dToVector3(vector3d));
               }

               setValue = message.getClass().getMethod(methodName, value.getClass());
               fieldValue = value;
            }
            else if (componentType.equals(Vector3f.class))
            {
               List<geometry_msgs.Vector3> value = new ArrayList<geometry_msgs.Vector3>();
               List<Vector3f> vectors = Arrays.asList((Vector3f[]) field.get(packet));

               for (Vector3f vector3f : vectors)
               {
                  value.add(convertVector3fToVector3(vector3f));
               }

               setValue = message.getClass().getMethod(methodName, value.getClass());
               fieldValue = value;
            }
            else if (componentType.equals(Quat4d.class))
            {
               List<geometry_msgs.Quaternion> value = new ArrayList<geometry_msgs.Quaternion>();
               List<Quat4d> quats = Arrays.asList((Quat4d[]) field.get(packet));

               for (Quat4d quat : quats)
               {
                  value.add(convertQuat4dToQuaternion(quat));
               }

               setValue = message.getClass().getMethod(methodName, value.getClass());
               fieldValue = value;
            }
            else
            {
               // Array of Objects J -> List of JMessage
               List<Message> value = new ArrayList<org.ros.internal.message.Message>();
               Object array = field.get(packet);
               int size = Array.getLength(array);

               for (int i = 0; i < size; i++){
                  value.add(convertToIHMCRosMessage(Array.get(array, i)));
               }

               setValue = message.getClass().getMethod(methodName, List.class);
               fieldValue = value;
            }
         }
         else if (List.class.isAssignableFrom(fieldType))
         {
            // field is a List
            Class componentType = ((Class) ((ParameterizedType) field.getGenericType()).getActualTypeArguments()[0]);

            if (componentType.equals(byte.class))
            {
               // Deal with ChannelBuffer set/get
               // must convert from byte[] to ChannelBuffer
               List<Byte> objects = (List<Byte>) field.get(packet);
               byte[] byteArray = new byte[objects.size()];

               for (int i = 0; i < objects.size(); i++)
               {
                  byteArray[i] = objects.get(i).byteValue();
               }

               ChannelBuffer value = ChannelBuffers.wrappedBuffer(byteArray);
               setValue = message.getClass().getMethod(methodName, value.getClass());
               fieldValue = value;
            }
            else if (componentType.isEnum())
            {
               // is List<ENUM>
               // will be going to uint8[] field, requires channelbuffer
               List<Enum> objects = (List<Enum>) field.get(packet);
               byte[] values = new byte[objects.size()];

               for (int i = 0; i < values.length; i++)
               {
                  values[i] = (byte) objects.get(i).ordinal();
               }

               setValue = message.getClass().getMethod(methodName, ChannelBuffer.class);
               fieldValue = ChannelBuffers.wrappedBuffer(values);
            }
            else if (isPrimativeWrapper(componentType))
            {
               Object value = 0;

               if (componentType.equals(Boolean.class))
               {
                  List<Boolean> objects = (List<Boolean>) field.get(packet);

                  value = ArrayUtils.toPrimitive(objects.toArray(new Boolean[objects.size()]));
               }
               else if (componentType.equals(Short.class))
               {
                  List<Short> objects = (List<Short>) field.get(packet);

                  value = ArrayUtils.toPrimitive(objects.toArray(new Short[objects.size()]));
               }
               else if (componentType.equals(Integer.class))
               {
                  List<Integer> objects = (List<Integer>) field.get(packet);

                  value = ArrayUtils.toPrimitive(objects.toArray(new Integer[objects.size()]));
               }
               else if (componentType.equals(Long.class))
               {
                  List<Long> objects = (List<Long>) field.get(packet);

                  value = ArrayUtils.toPrimitive(objects.toArray(new Long[objects.size()]));
               }
               else if (componentType.equals(Float.class))
               {
                  List<Float> objects = (List<Float>) field.get(packet);

                  value = ArrayUtils.toPrimitive(objects.toArray(new Float[objects.size()]));
               }
               else if (componentType.equals(Double.class))
               {
                  List<Double> objects = (List<Double>) field.get(packet);

                  value = ArrayUtils.toPrimitive(objects.toArray(new Double[objects.size()]));
               }

               setValue = message.getClass().getMethod(methodName, List.class);
               fieldValue = value;
            }
            else if (componentType.equals(String.class))
            {
               setValue = message.getClass().getMethod(methodName, List.class);
               fieldValue = field.get(packet);
            }
            else if (componentType.equals(Point3d.class))
            {
               List<geometry_msgs.Vector3> value = new ArrayList<geometry_msgs.Vector3>();
               List<Point3d> points = (List<Point3d>) field.get(packet);

               for (Point3d point : points)
               {
                  value.add(convertPoint3dToVector3(point));
               }

               setValue = message.getClass().getMethod(methodName, List.class);
               fieldValue = value;
            }
            else if (componentType.equals(Vector3d.class))
            {
               List<geometry_msgs.Vector3> value = new ArrayList<geometry_msgs.Vector3>();
               List<Vector3d> vectors = (List<Vector3d>) field.get(packet);

               for (Vector3d vector3d : vectors)
               {
                  value.add(convertVector3dToVector3(vector3d));
               }

               setValue = message.getClass().getMethod(methodName, List.class);
               fieldValue = value;
            }
            else if (componentType.equals(Vector3f.class))
            {
               List<geometry_msgs.Vector3> value = new ArrayList<geometry_msgs.Vector3>();
               List<Vector3f> vectors = (List<Vector3f>) field.get(packet);

               for (Vector3f vector3f : vectors)
               {
                  value.add(convertVector3fToVector3(vector3f));
               }

               setValue = message.getClass().getMethod(methodName, List.class);
               fieldValue = value;
            }
            else if (componentType.equals(Quat4d.class))
            {
               List<geometry_msgs.Quaternion> value = new ArrayList<geometry_msgs.Quaternion>();
               List<Quat4d> quats = (List<Quat4d>) field.get(packet);

               for (Quat4d quat : quats)
               {
                  value.add(convertQuat4dToQuaternion(quat));
               }

               setValue = message.getClass().getMethod(methodName, List.class);
               fieldValue = value;
            }
            else
            {
               // Array of Objects J -> List of JMessage
               List<?> objects = (List<?>) field.get(packet);
               List<Message> value = new ArrayList<Message>();

               for (Object object : objects)
               {
                  value.add(convertToIHMCRosMessage(object));
               }

               setValue = message.getClass().getMethod(methodName, List.class);
               fieldValue = value;
            }
         }
         else if (fieldType.equals(Point3d.class))
         {
            // message is geometry_msgs/Vector3
            setValue = message.getClass().getMethod(methodName, geometry_msgs.Vector3.class);
            fieldValue = convertPoint3dToVector3((Point3d) field.get(packet));
         }
         else if (fieldType.equals(Vector3d.class))
         {
            // message is geometry_msgs/Quaternion
            setValue = message.getClass().getMethod(methodName, geometry_msgs.Vector3.class);
            fieldValue = convertVector3dToVector3((Vector3d) field.get(packet));
         }
         else if (fieldType.equals(Vector3f.class))
         {
            // message is geometry_msgs/Quaternion
            setValue = message.getClass().getMethod(methodName, geometry_msgs.Vector3.class);
            fieldValue = convertVector3fToVector3((Vector3f) field.get(packet));
         }
         else if (fieldType.equals(Quat4d.class))
         {
            // message is geometry_msgs/Quaternion
            setValue = message.getClass().getMethod(methodName, geometry_msgs.Quaternion.class);
            fieldValue = convertQuat4dToQuaternion((Quat4d) field.get(packet));
         }
         else
         {
            // message is ihmc_msgs/(classname+Message)
            Message value = convertToIHMCRosMessage(field.get(packet));
            setValue = message.getClass().getMethod(methodName, (Class.forName((value.toRawMessage().getType()).replace('/', '.'))));
            fieldValue = value;
         }

         if ((setValue != null) && (fieldValue != null))
         {
            setValue.setAccessible(true);
            setValue.invoke(message, fieldValue);
         }
      }

      return message;
   }

   private static String getMethodNameForFieldName(String type, String fieldName)
   {
      String regex = "([a-z])([A-Z]+)";
      String replacement = "$1_$2";

      String inWithUnderscores = fieldName.replaceAll(regex, replacement);
      String ret = inWithUnderscores.toLowerCase();
      ret = type + "_" + ret;

      String methodName = CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, ret);
      return methodName;
   }

   public static void convertToPacket(Message message, Object outputPacket)
           throws IllegalArgumentException, IllegalAccessException, InvocationTargetException, NoSuchMethodException, SecurityException, InstantiationException
   {
      Field[] fields = outputPacket.getClass().getFields();

      for (Field field : fields)
      {
         if (Modifier.isFinal(field.getModifiers()) || isConstant(field) || isIgnoredField(field))
         {
            continue;
         }

         String fieldName = field.getName();
         String methodName = getMethodNameForFieldName("get", fieldName);
         Class fieldType = field.getType();
         Method getValue = message.getClass().getMethod(methodName);
         getValue.setAccessible(true);

         if (fieldType.isPrimitive() || fieldType.equals(String.class))
         {
            field.set(outputPacket, getValue.invoke(message));
         }
         else if (fieldType.isEnum())
         {
            // will be going to uint8 field
            int index = (byte) getValue.invoke(message);
            if ((index < 0) || (index >= fieldType.getEnumConstants().length))
            {
               // print a useful message to syserr for GFE teams:
               System.err.println(fieldName + " has invalid value of " + index + " (the size of enum " + fieldType.getName() + " is "
                                  + fieldType.getEnumConstants().length + ")");

               outputPacket = null;

               return;
            }
            else
            {
               field.set(outputPacket, (fieldType.getEnumConstants())[index]);
            }
         }
         else if (fieldType.isArray())
         {
            // will be in form field[] or List going to []
            Class componentType = fieldType.getComponentType();

            if (componentType.equals(byte.class))
            {
               // Deal with ChannelBuffer set/get
               // must convert from byte[] to ChannelBuffer
               ChannelBuffer buffer = (ChannelBuffer) getValue.invoke(message);
               field.set(outputPacket, buffer.array());
            }
            else if (componentType.isEnum())
            {
               // is ENUM[]
               // will be going from uint8[] field, requires channelbuffer
               ChannelBuffer buffer = (ChannelBuffer) getValue.invoke(message);
               byte[] indices = buffer.array();
               Object[] enumValues = componentType.getEnumConstants();
               Enum[] values = new Enum[indices.length];
               for (int i = 0; i < indices.length; i++)
               {
                  values[i] = (Enum) enumValues[indices[i]];
               }

               field.set(outputPacket, values);
            }
            else if (componentType.isPrimitive())
            {
               // no conversion
               field.set(outputPacket, getValue.invoke(message));
            }
            else if (componentType.equals(String.class))
            {
               // comes in as List, must leave as Array
               field.set(outputPacket, ((List) getValue.invoke(message)).get(0));
            }
            else if (componentType.equals(Point3d.class))
            {
               List<Vector3> vectors = (List) getValue.invoke(message);
               List<Point3d> points = new ArrayList<Point3d>();

               for (Vector3 vector3 : vectors)
               {
                  points.add(convertVector3ToPoint3d(vector3));
               }

               field.set(outputPacket, points.toArray());
            }
            else if (fieldType.equals(Vector3d.class))
            {
               List<Vector3> vectors = (List) getValue.invoke(message);
               List<Vector3d> points = new ArrayList<Vector3d>();

               for (Vector3 vector3 : vectors)
               {
                  points.add(convertVector3ToVector3d(vector3));
               }

               field.set(outputPacket, points.toArray());
            }
            else if (fieldType.equals(Vector3f.class))
            {
               List<Vector3> vectors = (List) getValue.invoke(message);
               List<Vector3f> points = new ArrayList<Vector3f>();

               for (Vector3 vector3 : vectors)
               {
                  points.add(convertVector3ToVector3f(vector3));
               }

               field.set(outputPacket, points.toArray());
            }
            else if (componentType.equals(Quat4d.class))
            {
               List<Quaternion> quaternions = (List) getValue.invoke(message);
               List<Quat4d> quat4ds = new ArrayList<Quat4d>();

               for (Quaternion quaternion : quaternions)
               {
                  quat4ds.add(convertQuaternionToQuat4d(quaternion));
               }

               field.set(outputPacket, quat4ds.toArray());

            }
            else
            {
               // List of JMessage -> Array of Objects J
               List<Message> objects = (List) getValue.invoke(message);

               Object valueArray = Array.newInstance(componentType, objects.size());
               int index = 0;
               for (Message object : objects)
               {
                  // create new object of ComonentType
                  Object value = componentType.newInstance();

                  // set from message
                  convertToPacket(object, value);
                  Array.set(valueArray, index, value);
                  index++;
               }

               {
                  field.set(outputPacket, valueArray);
               }

            }
         }
         else if (List.class.isAssignableFrom(fieldType))
         {
            // field is a List
            Class componentType = ((Class) ((ParameterizedType) field.getGenericType()).getActualTypeArguments()[0]);
            List valueList;

            if (componentType.equals(Byte.class))
            {
               // Deal with ChannelBuffer set/get
               // must convert from byte[] to ChannelBuffer
               ChannelBuffer buffer = (ChannelBuffer) getValue.invoke(message);
               valueList = Arrays.asList(buffer.array());
            }
            else if (componentType.isEnum())
            {
               // will be going from uint8[] field, requires channelbuffer
               ChannelBuffer buffer = (ChannelBuffer) getValue.invoke(message);
               byte[] indices = buffer.array();
               Object[] enumValues = componentType.getEnumConstants();
               Enum[] values = new Enum[indices.length];
               for (int i = 0; i < indices.length; i++)
               {
                  values[i] = (Enum) enumValues[indices[i]];
               }

               valueList = Arrays.asList(values);
            }
            else if (componentType.isPrimitive() || isPrimativeWrapper(componentType))
            {
               // no conversion
               valueList = Arrays.asList(getValue.invoke(message));
            }
            else if (componentType.equals(String.class))
            {
               // comes in as List, must leave as Array
               valueList = (List) (getValue.invoke(message));
            }
            else if (componentType.equals(Point3d.class))
            {
               List<Vector3> vectors = (List) getValue.invoke(message);
               List<Point3d> points = new ArrayList<Point3d>();

               for (Vector3 vector3 : vectors)
               {
                  points.add(convertVector3ToPoint3d(vector3));
               }

               valueList = points;
            }
            else if (fieldType.equals(Vector3d.class))
            {
               List<Vector3> vectors = (List) getValue.invoke(message);
               List<Vector3d> points = new ArrayList<Vector3d>();

               for (Vector3 vector3 : vectors)
               {
                  points.add(convertVector3ToVector3d(vector3));
               }

               valueList = points;
            }
            else if (fieldType.equals(Vector3f.class))
            {
               List<Vector3> vectors = (List) getValue.invoke(message);
               List<Vector3f> points = new ArrayList<Vector3f>();

               for (Vector3 vector3 : vectors)
               {
                  points.add(convertVector3ToVector3f(vector3));
               }

               valueList = points;
            }
            else if (componentType.equals(Quat4d.class))
            {
               List<Quaternion> quaternions = (List) getValue.invoke(message);
               List<Quat4d> quat4ds = new ArrayList<Quat4d>();

               for (Quaternion quaternion : quaternions)
               {
                  quat4ds.add(convertQuaternionToQuat4d(quaternion));
               }

               valueList = quat4ds;
            }
            else
            {
               // List of JMessage -> Array of Objects J
               List<Message> objects = (List) getValue.invoke(message);

               List values = new ArrayList<>();

               for (Message object : objects)
               {
                  // create new object of ComonentType
                  Object value = componentType.newInstance();

                  // set from message
                  convertToPacket(object, value);
                  values.add(value);

               }

               valueList = values;
            }

            if (valueList.size() != 0)
            {
               field.set(outputPacket, valueList);
            }
         }
         else if (fieldType.equals(Point3d.class))
         {
            field.set(outputPacket, convertVector3ToPoint3d((Vector3) getValue.invoke(message)));
         }
         else if (fieldType.equals(Vector3d.class))
         {
            field.set(outputPacket, convertVector3ToVector3d((Vector3) getValue.invoke(message)));
         }
         else if (fieldType.equals(Vector3f.class))
         {
            field.set(outputPacket, convertVector3ToVector3f((Vector3) getValue.invoke(message)));
         }
         else if (fieldType.equals(Quat4d.class))
         {
            field.set(outputPacket, convertQuaternionToQuat4d((Quaternion) getValue.invoke(message)));
         }
         else
         {
            // message is ihmc_msgs/(classname+Message)

            // create new object of ComonentType
            Object value = fieldType.newInstance();

            // set from message
            convertToPacket((Message) getValue.invoke(message), value);
            field.set(outputPacket, value);
         }
      }

      return;
   }

   public static geometry_msgs.Vector3 convertPoint3dToVector3(Point3d point)
   {
      geometry_msgs.Vector3 value = messageFactory.newFromType("geometry_msgs/Vector3");

      value.setX(point.getX());
      value.setY(point.getY());
      value.setZ(point.getZ());

      return value;
   }


   public static Vector3 convertVector3dToVector3(Vector3d vector3d)
   {
      geometry_msgs.Vector3 value = messageFactory.newFromType("geometry_msgs/Vector3");

      value.setX(vector3d.getX());
      value.setY(vector3d.getY());
      value.setZ(vector3d.getZ());

      return value;
   }
   public static Vector3 convertVector3fToVector3(Vector3f vector3f){
      geometry_msgs.Vector3 value = messageFactory.newFromType("geometry_msgs/Vector3");

      value.setX(vector3f.getX());
      value.setY(vector3f.getY());
      value.setZ(vector3f.getZ());

      return value;
   }

   public static FramePose convertPoseStampedToFramePose(ReferenceFrame referenceFrame, Pose msg)
   {
      Point point = msg.getPosition();
      Quat4d quat = convertQuaternionToQuat4d(msg.getOrientation());
      double[] yawPitchRoll = new double[3];
      RotationTools.convertQuaternionToYawPitchRoll(quat, yawPitchRoll);

      FramePoint position = new FramePoint(referenceFrame, point.getX(), point.getY(), point.getZ());
      FrameOrientation orientation = new FrameOrientation(referenceFrame, yawPitchRoll);

      return new FramePose(position, orientation);
   }

   public static geometry_msgs.Quaternion convertQuat4dToQuaternion(Quat4d quat)
   {
      geometry_msgs.Quaternion value = messageFactory.newFromType("geometry_msgs/Quaternion");

      value.setX(quat.getX());
      value.setY(quat.getY());
      value.setZ(quat.getZ());
      value.setW(quat.getW());

      return value;
   }

   public static Point3d convertVector3ToPoint3d(geometry_msgs.Vector3 vector)
   {
      Point3d point = new Point3d(vector.getX(), vector.getY(), vector.getZ());

      return point;
   }

   public static Vector3d convertVector3ToVector3d(geometry_msgs.Vector3 vector)
   {
      Vector3d vector3d = new Vector3d(vector.getX(), vector.getY(), vector.getZ());

      return vector3d;
   }

   public static Vector3f convertVector3ToVector3f(geometry_msgs.Vector3 vector)
   {
      Vector3f vector3f = new Vector3f((float) vector.getX(), (float)vector.getY(), (float)vector.getZ());

      return vector3f;
   }

   public static Quat4d convertQuaternionToQuat4d(geometry_msgs.Quaternion quaternion)
   {
      Quat4d quat4d = new Quat4d(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());

      return quat4d;
   }

   public static boolean isPrimativeWrapper(Class type)
   {
      return (type.equals(Boolean.class) || type.equals(Byte.class) || type.equals(Short.class) || type.equals(Integer.class) || type.equals(Long.class)
              || type.equals(Float.class) || type.equals(Double.class));
   }

   private static boolean isConstant(Field field)
   {
      int modifier = field.getModifiers();

      return Modifier.isFinal(modifier) && Modifier.isPublic(modifier) && Modifier.isStatic(modifier);
   }

   private static boolean isIgnoredField(Field field)
   {
      return field.isAnnotationPresent(IgnoreField.class);
   }
   
   public static <E extends Enum<E>> E convertByteToEnum(Class<E> enumClass, byte ordinal)
   {
      return enumClass.getEnumConstants()[ordinal];
   }
   
   public static byte convertEnumToByte(Enum<?> val)
   {
      return (byte)val.ordinal();
   }
}
