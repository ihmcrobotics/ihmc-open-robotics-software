package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.lang.reflect.Field;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.idl.PreallocatedList;

/**
 * This is an highly inefficient way of enabling the transformation of messages using a
 * {@code RigidBodyTransform}.
 * <p>
 * This class was created to remove custom features implemented in messages that were including a
 * transformation method. This class should be used as little as possible and eventually deleted.
 * </p>
 */
public final class MessageTransformer
{
   private MessageTransformer()
   {
   }

   /**
    * Recursively transforms the fields, including sub-fields, that implement {@code Transformable}
    * of the given {@code message}.
    * <p>
    * Some types are handling in a specific manner as {@code AdjustFootstepMessage} and
    * {@code FootstepDataMessage}.
    * </p>
    * 
    * @param message the object to be transformed. Modified.
    * @param rigidBodyTransformToApply the transform to apply on the given message. Not modified.
    */
   @SuppressWarnings("rawtypes")
   public static void transform(Object message, RigidBodyTransform rigidBodyTransformToApply)
   {
      if (message == null || isPrimitive(message.getClass()))
         return;

      if (message instanceof AdjustFootstepMessage)
      {
         transform((AdjustFootstepMessage) message, rigidBodyTransformToApply);
      }
      else if (message instanceof FootstepDataMessage)
      {
         transform((FootstepDataMessage) message, rigidBodyTransformToApply);
      }
      else if (message instanceof PreallocatedList)
      {
         PreallocatedList list = (PreallocatedList) message;
         for (int i = 0; i < list.size(); i++)
            transform(list.get(i), rigidBodyTransformToApply);
      }
      else if (message instanceof Iterable)
      {
         for (Object element : (Iterable) message)
            transform(element, rigidBodyTransformToApply);
      }
      else if (message.getClass().isArray())
      {
         for (Object element : (Object[]) message)
            transform(element, rigidBodyTransformToApply);
      }
      else
      {
         if (message instanceof Transformable)
         {
            ((Transformable) message).applyTransform(rigidBodyTransformToApply);
            return;
         }

         if (Enum.class.isAssignableFrom(message.getClass()))
            return;

         Field[] fields = message.getClass().getFields();

         for (Field field : fields)
         {
            try
            {
               transform(field.get(message), rigidBodyTransformToApply);
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               e.printStackTrace();
               return;
            }
         }
      }
   }

   private static void transform(AdjustFootstepMessage message, RigidBodyTransform rigidBodyTransformToApply)
   {
      if (message.location != null)
         message.location.applyTransform(rigidBodyTransformToApply);
      if (message.orientation != null)
         message.orientation.applyTransform(rigidBodyTransformToApply);
   }

   private static void transform(FootstepDataMessage message, RigidBodyTransform rigidBodyTransformToApply)
   {
      if (message.location != null)
         message.location.applyTransform(rigidBodyTransformToApply);
      if (message.orientation != null)
         message.orientation.applyTransform(rigidBodyTransformToApply);

      if (message.positionWaypoints != null)
      {
         for (int i = 0; i < message.positionWaypoints.size(); i++)
            message.positionWaypoints.get(i).applyTransform(rigidBodyTransformToApply);
      }
   }

   private static boolean isPrimitive(Class<?> clazz)
   {
      if (clazz.equals(boolean.class) || clazz.equals(boolean[].class) || clazz.equals(Boolean.class))
         return true;
      if (clazz.equals(byte.class) || clazz.equals(byte[].class) || clazz.equals(Byte.class))
         return true;
      if (clazz.equals(char.class) || clazz.equals(char[].class) || clazz.equals(Character.class))
         return true;
      if (clazz.equals(short.class) || clazz.equals(short[].class) || clazz.equals(Short.class))
         return true;
      if (clazz.equals(int.class) || clazz.equals(int[].class) || clazz.equals(Integer.class))
         return true;
      if (clazz.equals(long.class) || clazz.equals(long[].class) || clazz.equals(Long.class))
         return true;
      if (clazz.equals(float.class) || clazz.equals(float[].class) || clazz.equals(Float.class))
         return true;
      if (clazz.equals(double.class) || clazz.equals(double[].class) || clazz.equals(Double.class))
         return true;
      return false;
   }
}
