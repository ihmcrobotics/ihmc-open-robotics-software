package us.ihmc.sensorProcessing.frames;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.variable.frameObjects.FrameIndexMap;

/**
 * This class represents a map to retrieve reference frame from their hash-code.
 * <p>
 * Utilities are provided to extract reference frames from {@code FullRobotModel} and
 * {@code ReferenceFrames}.
 * </p>
 * <p>
 * Reference frames should be registered at construction and leave this map unchanging at runtime.
 * </p>
 * 
 * @author Brandon Shrewsbury
 */
public class ReferenceFrameHashCodeResolver implements FrameIndexMap
{
   /** Default hash code for any object that is equal to {@code null}. */
   public static final long NULL_HASHCODE = 0L;
   private static final boolean DEBUG = false;

   /** The internal map storage. */
   private final TLongObjectHashMap<ReferenceFrame> hashCodeToReferenceFrameMap = new TLongObjectHashMap<>();

   /**
    * Creates a new resolver with only the {@code null} frame and
    * {@link ReferenceFrame#getWorldFrame()} registered.
    */
   public ReferenceFrameHashCodeResolver()
   {
      hashCodeToReferenceFrameMap.put(NULL_HASHCODE, null);
      put(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new resolver and sets it up to map all the reference frames declared in the given
    * {@code fullRobotModel} and {@code referenceFrames}.
    * <p>
    * The {@code null} frame and {@link ReferenceFrame#getWorldFrame()} are also registered.
    * </p>
    * 
    * @param fullRobotModel the robot model from which reference frames should be registered.
    * @param referenceFrames the frame holder from which reference frames should be registered.
    */
   public ReferenceFrameHashCodeResolver(FullRobotModel fullRobotModel, ReferenceFrames referenceFrames)
   {
      this();
      putAllFullRobotModelReferenceFrames(fullRobotModel);
      putAllReferenceFrames(referenceFrames);
   }

   /**
    * Creates a new resolver and registers all the given {@code referenceFrames}.
    * <p>
    * The {@code null} frame and {@link ReferenceFrame#getWorldFrame()} are also registered.
    * </p>
    * 
    * @param referenceFrames the reference frames to be registered.
    */
   public ReferenceFrameHashCodeResolver(List<ReferenceFrame> referenceFrames)
   {
      this();
      putAll(referenceFrames);
   }

   /**
    * Registers a new reference frame into this map.
    * <p>
    * In the case the given {@code referenceFrame} was already registered, this map is not modified.
    * </p>
    * 
    * @param referenceFrame the reference frame to register.
    * @throws IllegalArgumentException if a distinct reference frame was already registered under the
    *            same hash-code as {@code referenceFrame.hashCode()}. This usually occurs if: 1- the
    *            two reference frames are two distinct instances with the same name, 2- (very unlikely)
    *            bad luck and the hash-code algorithm generated the same hash-code for the two
    *            reference frames.
    */
   @Override
   public void put(ReferenceFrame referenceFrame)
   {
      put(referenceFrame, referenceFrame.hashCode());
   }

   /**
    * Registers a new reference frame into this map with a custom hash-code.
    * <p>
    * In the case the given {@code referenceFrame} was already registered with the same hash-code, this
    * map is not modified.
    * </p>
    * 
    * @param referenceFrame the reference frame to register.
    * @param frameHashCode the custom hash-code to associate with the given frame.
    * @throws IllegalArgumentException if a distinct reference frame was already registered under the
    *            same hash-code as {@code referenceFrame.hashCode()}.
    */
   public void put(ReferenceFrame referenceFrame, long frameHashCode)
   {
      if (hashCodeToReferenceFrameMap.containsKey(frameHashCode))
      {
         ReferenceFrame existingFrame = hashCodeToReferenceFrameMap.get(frameHashCode);
         if (referenceFrame != existingFrame)
         {
            throw new IllegalArgumentException(getClass().getSimpleName()
                  + ": The reference frame has the same hash-code as another distinct reference frame previously registered.");
         }
      }
      else
      {
         hashCodeToReferenceFrameMap.put(frameHashCode, referenceFrame);
      }
   }

   /**
    * Extracts then registers all the reference frames associated to the given {@code fullRobotModel}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * In the case one of the reference frames of the given {@code fullRobotModel} was already
    * registered, this map is not modified.
    * </p>
    * 
    * @param fullRobotModel the robot model to register the reference frames of.
    * @throws IllegalArgumentException in the case of a hash-code collision, i.e. two distinct
    *            reference frames with the same hash-code.
    */
   public void putAllFullRobotModelReferenceFrames(FullRobotModel fullRobotModel)
   {
      putAllMultiBodySystemReferenceFrames(fullRobotModel.getElevator());
      putAll(extractReferenceFrames(fullRobotModel));
   }

   /**
    * Extracts the registers all the reference frames attached to the joints and rigid-bodies of the
    * multi-body system attached to the given {@code rootBody}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * In the case one of the reference frame of the multi-body system was already registered, this map
    * is not modified.
    * </p>
    * 
    * @param rootBody the root of the multi-body system to register the reference frames of.
    * @throws IllegalArgumentException if a distinct reference frame was already registered under the
    *            same hash-code as one of the multi-body system's frame or if two or more distinct
    *            reference frames from the multi-body system share the same hash-code. This usually
    *            occurs if: 1- the two reference frame are two distinct instances with the same name,
    *            2- (very unlikely) bad luck and the hash-code algorithm generated the same hash-code
    *            for the two reference frames.
    */
   public void putAllMultiBodySystemReferenceFrames(RigidBodyReadOnly rootBody)
   {
      putAll(extractMultiBodySystemReferenceFrames(rootBody));
   }

   /**
    * Extracts then registers all the reference frames that are defined in the given
    * {@code referenceFrames}, also registers if possible the listed reference frames with custom IDs,
    * i.e. in the case {@link ReferenceFrames#getReferenceFrameDefaultHashIds()} does not return
    * {@code null}.
    * <p>
    * WARNING: This method uses reflection and generates garbage.
    * </p>
    * <p>
    * In the case one of the reference frame of the given {@code referenceFrames} was already
    * registered, this map is not modified.
    * </p>
    * <p>
    * The reference frames are extracted using reflection and rely on the class to declare getters for
    * each individual frame.
    * </p>
    * 
    * @param referenceFrames the reference frame holder to register the reference frames of.
    * @throws IllegalArgumentException if a distinct reference frame was already registered under the
    *            same hash-code as one of the {@code referenceFrames}'s frame or if two or more
    *            distinct reference frames from the {@code referenceFrames} share the same hash-code.
    *            This usually occurs if: 1- the two reference frame are two distinct instances with the
    *            same name, 2- (very unlikely) bad luck and the hash-code algorithm generated the same
    *            hash-code for the two reference frames.
    */
   public void putAllReferenceFrames(ReferenceFrames referenceFrames)
   {
      putAll(extractReferenceFrames(referenceFrames));
      registerCustomReferenceFrameIds(referenceFrames.getReferenceFrameDefaultHashIds());
   }

   /**
    * Navigates the given map and registers the different reference frames with a given custom ID.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param customIdsToReferenceFrameMap the map from custom ID to reference frame.
    * @throws IllegalArgumentException if a distinct reference frame was already registered under the
    *            same hash-code.
    */
   public void registerCustomReferenceFrameIds(TLongObjectMap<ReferenceFrame> customIdsToReferenceFrameMap)
   {
      if (customIdsToReferenceFrameMap == null)
         return;

      for (long key : customIdsToReferenceFrameMap.keys())
      {
         ReferenceFrame referenceFrame = customIdsToReferenceFrameMap.get(key);

         if (referenceFrame != null)
         {
            put(referenceFrame);
            put(referenceFrame, key);
         }
      }
   }

   @Deprecated
   public ReferenceFrame getReferenceFrameFromHashCode(long frameHashCode)
   {
      return getReferenceFrame(frameHashCode);
   }

   /**
    * Gets the reference frame associated to the given {@code frameHashCode}.
    * 
    * @param frameHashCode the hash-code used to retrieve the reference frame, it is usually generated
    *           from {@code referenceFrame.hashCode()}.
    * @return the corresponding reference frame.
    * @throws RuntimeException if no reference frame is associated to the given hash-code.
    */
   @Override
   public ReferenceFrame getReferenceFrame(long frameHashCode)
   {
      if (!hashCodeToReferenceFrameMap.containsKey(frameHashCode))
         throw new RuntimeException("Unknown reference frame.");

      return hashCodeToReferenceFrameMap.get(frameHashCode);
   }

   /**
    * Gets the hash-code associated to the given reference frame.
    * <p>
    * This is equivalent to {@link ReferenceFrame#hashCode()} excepts that this getter asserts that the
    * reference frame is already register into this map.
    * </p>
    * 
    * @return the corresponding hash-code.
    * @throws RuntimeException if the reference frame is not part of this map.
    */
   @Override
   public long getFrameIndex(ReferenceFrame referenceFrame)
   {
      long hashCode = referenceFrame.hashCode();

      if (!hashCodeToReferenceFrameMap.contains(hashCode))
         throw new RuntimeException("Unknown reference frame.");

      return hashCode;
   }

   /**
    * Gets all the reference frames that has been registered to this map.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the collection of all this map's reference frames.
    */
   public Collection<ReferenceFrame> getAllReferenceFrames()
   {
      return hashCodeToReferenceFrameMap.valueCollection();
   }

   /**
    * Extracts all the reference frames attached to the joints and rigid-bodies of the multi-body
    * system attached to the given {@code rootBody}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param rootBody the root of the multi-body system to extract the reference frames from.
    * @return the list of all the extracted reference frames.
    */
   public static List<ReferenceFrame> extractMultiBodySystemReferenceFrames(RigidBodyReadOnly rootBody)
   {
      List<ReferenceFrame> extractedReferenceFrames = new ArrayList<>();

      extractedReferenceFrames.add(rootBody.getBodyFixedFrame().getParent());

      for (RigidBodyReadOnly rigidBody : rootBody.subtreeIterable())
      {
         extractedReferenceFrames.add(rigidBody.getBodyFixedFrame());
      }

      for (JointReadOnly joint : rootBody.childrenSubtreeIterable())
      {
         extractedReferenceFrames.add(joint.getFrameAfterJoint());
         extractedReferenceFrames.add(joint.getFrameBeforeJoint());
      }

      return extractedReferenceFrames;
   }

   /**
    * Extracts using reflection the reference frames from a reference frame holder.
    * <p>
    * WARNING: This method uses reflection and generates garbage.
    * </p>
    * <p>
    * This methods attempts to invoke any method of the given {@code referenceFrameHolder} that returns
    * an instance of {@code ReferenceFrame}.
    * </p>
    * 
    * @param referenceFrameHolder the object to get the reference frames from.
    * @return the list of all the extracted reference frames.
    */
   public static List<ReferenceFrame> extractReferenceFrames(Object referenceFrameHolder)
   {
      Class<?> clazz = referenceFrameHolder.getClass();
      Method[] declaredMethods = clazz.getMethods();
      List<ReferenceFrame> extractedReferenceFrames = new ArrayList<>();

      for (Method method : declaredMethods)
      {
         if (ReferenceFrame.class.isAssignableFrom(method.getReturnType()))
         {
            if (method.getParameterCount() == 0)
            {
               ReferenceFrame referenceFrame = invokeReferenceFrameGetter(method, referenceFrameHolder);

               if (referenceFrame != null)
                  extractedReferenceFrames.add(referenceFrame);
            }
            else if (method.getParameterCount() == 1)
            {
               Class<?> parameterType = method.getParameterTypes()[0];

               if (parameterType == RobotSide.class)
               {
                  for (RobotSide robotSide : RobotSide.values)
                  {
                     ReferenceFrame referenceFrame = invokeReferenceFrameGetter(method, referenceFrameHolder, robotSide);

                     if (referenceFrame != null)
                        extractedReferenceFrames.add(referenceFrame);
                  }
               }
               else if (parameterType == RobotQuadrant.class)
               {
                  for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
                  {
                     ReferenceFrame referenceFrame = invokeReferenceFrameGetter(method, referenceFrameHolder, robotQuadrant);
                     if (referenceFrame != null)
                        extractedReferenceFrames.add(referenceFrame);
                  }
               }
               else if (parameterType == RobotSextant.class)
               {
                  for (RobotSextant robotSextant : RobotSextant.values)
                  {
                     ReferenceFrame referenceFrame = invokeReferenceFrameGetter(method, referenceFrameHolder, robotSextant);
                     if (referenceFrame != null)
                        extractedReferenceFrames.add(referenceFrame);
                  }
               }
               else if (DEBUG)
               {
                  LogTools.warn("Missing implementation to handle the method {} from the class {}.", method.getName(),
                                referenceFrameHolder.getClass().getSimpleName());
               }
            }
            else if (DEBUG)
            {
               LogTools.warn("Missing implementation to handle the method {} from the class {}.", method.getName(),
                             referenceFrameHolder.getClass().getSimpleName());
            }
         }
      }

      return extractedReferenceFrames;
   }

   private static ReferenceFrame invokeReferenceFrameGetter(Method method, Object getterHolder, Object... methodArguments)
   {
      try
      {
         return (ReferenceFrame) method.invoke(getterHolder, methodArguments);
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         LogTools.warn("Encountered problem invoking method {}  on {}. Error message:\n{}", method.getName(), getterHolder.getClass().getSimpleName(),
                       e.getMessage());
         return null;
      }
   }
}
