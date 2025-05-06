package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryMessage;
import geometry_msgs.msg.dds.Wrench;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * This is an highly inefficient way of enabling the transformation of messages using a
 * {@code RigidBodyTransform}.
 * <p>
 * This class was created to remove custom features implemented in messages that were including a
 * transformation method. This class should be used as little as possible and eventually deleted.
 * </p>
 */
@SuppressWarnings("rawtypes")
public final class MessageTransformer
{
   private static final Map<Class, CustomTransformer> customTransformers = createCustomTransformers();

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
   @SuppressWarnings({"unchecked"})
   public static void transform(Object message, RigidBodyTransform rigidBodyTransformToApply)
   {
      if (message == null || isPrimitive(message.getClass()))
         return;

      if (customTransformers.containsKey(message.getClass()))
      {
         customTransformers.get(message.getClass()).transform(message, rigidBodyTransformToApply);
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

   private static Map<Class, CustomTransformer> createCustomTransformers()
   {
      Map<Class, CustomTransformer> customTransformers = new HashMap<>();

      customTransformers.put(FootstepDataMessage.class, new CustomTransformer<FootstepDataMessage>()
      {
         @Override
         public void transform(FootstepDataMessage message, RigidBodyTransform rigidBodyTransformToApply)
         {
            if (message.getLocation() != null)
               message.getLocation().applyTransform(rigidBodyTransformToApply);
            if (message.getOrientation() != null)
               message.getOrientation().applyTransform(rigidBodyTransformToApply);

            if (message.getCustomPositionWaypoints() != null)
            {
               for (int i = 0; i < message.getCustomPositionWaypoints().size(); i++)
                  message.getCustomPositionWaypoints().get(i).applyTransform(rigidBodyTransformToApply);
            }
         }
      });

      customTransformers.put(SE3TrajectoryMessage.class, new CustomTransformer<SE3TrajectoryMessage>()
      {
         @Override
         public void transform(SE3TrajectoryMessage object, RigidBodyTransform rigidBodyTransformToApply)
         {
            MessageTransformer.transform(object.getTaskspaceTrajectoryPoints(), rigidBodyTransformToApply);
         }
      });

      customTransformers.put(SO3TrajectoryMessage.class, new CustomTransformer<SO3TrajectoryMessage>()
      {
         @Override
         public void transform(SO3TrajectoryMessage object, RigidBodyTransform rigidBodyTransformToApply)
         {
            MessageTransformer.transform(object.getTaskspaceTrajectoryPoints(), rigidBodyTransformToApply);
         }
      });

      customTransformers.put(EuclideanTrajectoryMessage.class, new CustomTransformer<EuclideanTrajectoryMessage>()
      {
         @Override
         public void transform(EuclideanTrajectoryMessage object, RigidBodyTransform rigidBodyTransformToApply)
         {
            MessageTransformer.transform(object.getTaskspaceTrajectoryPoints(), rigidBodyTransformToApply);
         }
      });

      customTransformers.put(WrenchTrajectoryMessage.class, new CustomTransformer<WrenchTrajectoryMessage>()
      {
         @Override
         public void transform(WrenchTrajectoryMessage object, RigidBodyTransform rigidBodyTransformToApply)
         {
            MessageTransformer.transform(object.getWrenchTrajectoryPoints(), rigidBodyTransformToApply);
         }
      });

      customTransformers.put(Wrench.class, new CustomTransformer<Wrench>()
      {
         @Override
         public void transform(Wrench object, RigidBodyTransform rigidBodyTransformToApply)
         {
            object.getForce().applyTransform(rigidBodyTransformToApply);
            object.getTorque().applyTransform(rigidBodyTransformToApply);
         }
      });

      return customTransformers;
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

   private static interface CustomTransformer<T>
   {
      void transform(T object, RigidBodyTransform rigidBodyTransformToApply);
   }
}
