package us.ihmc.robotModels;

import java.util.Collection;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

/**
 * This class represents a map to retrieve joints from their hash-code.
 * <p>
 * Utilities are provided to extract joints from {@code FullRobotModel}.
 * </p>
 * <p>
 * Joints should be registered at construction and leave this map unchanging at runtime.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class JointHashCodeResolver
{
   /** Default hash code for any object that is equal to {@code null}. */
   public static final long NULL_HASHCODE = 0L;

   /** The internal map storage. */
   private final TLongObjectHashMap<JointReadOnly> hashCodeToJointMap = new TLongObjectHashMap<>();

   /**
    * Creates a new resolver with only the {@code null} joint registered.
    */
   public JointHashCodeResolver()
   {
      hashCodeToJointMap.put(NULL_HASHCODE, null);
   }

   /**
    * Creates a new resolver and sets it up to map all the joints declared in the given
    * {@code fullRobotModel}.
    * <p>
    * The {@code null} joint is also registered.
    * </p>
    * 
    * @param fullRobotModel the robot model from which joints should be registered.
    */
   public JointHashCodeResolver(FullRobotModel fullRobotModel)
   {
      this();
      putAllFullRobotModelJoints(fullRobotModel);
   }

   /**
    * Registers a new joint into this map.
    * <p>
    * In the case the given {@code joint} was already registered, this map is not modified.
    * </p>
    * 
    * @param joint the joint to register.
    * @throws IllegalArgumentException if a distinct joint was already registered under the same
    *            hash-code as {@code joint.hashCode()}. This usually occurs if: 1- the two joints are
    *            two distinct instances with the same name, 2- (very unlikely) bad luck and the
    *            hash-code algorithm generated the same hash-code for the two joints.
    */
   public void put(JointReadOnly joint)
   {
      put(joint, joint.hashCode());
   }

   /**
    * Registers a new joint into this map with a custom hash-code.
    * <p>
    * In the case the given {@code joint} was already registered with the same hash-code, this map is
    * not modified.
    * </p>
    * 
    * @param joint the joint to register.
    * @param jointHashCode the custom hash-code to associate with the given joint.
    * @throws IllegalArgumentException if a distinct joint was already registered under the same
    *            hash-code as {@code joint.hashCode()}.
    */
   public void put(JointReadOnly joint, long jointHashCode)
   {
      if (hashCodeToJointMap.containsKey(jointHashCode))
      {
         JointReadOnly existingJoint = hashCodeToJointMap.get(jointHashCode);
         if (joint != existingJoint)
         {
            throw new IllegalArgumentException(getClass().getSimpleName()
                  + ": The joint has the same hash-code as another distinct joint previously registered.");
         }
      }
      else
      {
         hashCodeToJointMap.put(jointHashCode, joint);
      }
   }

   /**
    * Extracts then registers all the joints associated to the given {@code fullRobotModel}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * In the case one of the joints of the given {@code fullRobotModel} was already registered, this
    * map is not modified.
    * </p>
    * 
    * @param fullRobotModel the robot model to register the joints of.
    * @throws IllegalArgumentException in the case of a hash-code collision, i.e. two distinct joints
    *            with the same hash-code.
    */
   public void putAllFullRobotModelJoints(FullRobotModel fullRobotModel)
   {
      putAllMultiBodySystemJoints(fullRobotModel.getElevator());
   }

   /**
    * Extracts then registers all the joints associated to the multi-body system attached to the given
    * {@code rootBody}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * In the case one of the joints of the multi-body system was already registered, this map is not
    * modified.
    * </p>
    * 
    * @param rootBody the root of the multi-body system to register the joints of.
    * @throws IllegalArgumentException in the case of a hash-code collision, i.e. two distinct joints
    *            with the same hash-code.
    */
   public void putAllMultiBodySystemJoints(RigidBodyReadOnly rootBody)
   {
      rootBody.childrenSubtreeIterable().forEach(this::put);
   }

   /**
    * Extracts then registers all the joints associated to the multi-body system attached to the given
    * {@code rootJoint}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * In the case one of the joints of the multi-body system was already registered, this map is not
    * modified.
    * </p>
    * 
    * @param rootJoint the root of the multi-body system to register the joints of.
    * @throws IllegalArgumentException in the case of a hash-code collision, i.e. two distinct joints
    *            with the same hash-code.
    */
   public void putAllMultiBodySystemJoints(JointReadOnly rootJoint)
   {
      putAll(rootJoint.subtreeList());
   }

   /**
    * Registers a collection of new joints into this map.
    * <p>
    * In the case a joint of {@code joints} was already registered, this map is not modified.
    * </p>
    * 
    * @param joints the new joints to register.
    * @throws IllegalArgumentException in the case of a hash-code collision, i.e. two distinct joints
    *            with the same hash-code.
    */
   public void putAll(Collection<? extends JointReadOnly> joints)
   {
      joints.forEach(this::put);
   }

   /**
    * Navigates the given map and registers the different joints with a given custom ID.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param customIdsToJointMap the map from custom ID to joint.
    * @throws IllegalArgumentException if a distinct joint was already registered under the same
    *            hash-code.
    */
   public void registerCustomJointIds(TLongObjectMap<? extends JointReadOnly> customIdsToJointMap)
   {
      if (customIdsToJointMap == null)
         return;

      for (long key : customIdsToJointMap.keys())
      {
         JointReadOnly joint = customIdsToJointMap.get(key);

         if (joint != null)
         {
            put(joint);
            put(joint, key);
         }
      }
   }

   /**
    * Casts then gets the joint associated to the given hash-code.
    * 
    * @param jointHashCode the hash-code used to retrieve the joint, it is usually generated from
    *           {@code joint.hashCode()}.
    * @return the corresponding joint casted.
    * @param <J> the type the joint is casted to.
    */
   @SuppressWarnings("unchecked")
   public <J extends JointReadOnly> J castAndGetJoint(long jointHashCode)
   {
      return (J) getJoint(jointHashCode);
   }

   /**
    * Gets the joint associated to the given hash-code.
    * 
    * @param jointHashCode the hash-code used to retrieve the joint, it is usually generated from
    *           {@code joint.hashCode()}.
    * @return the corresponding joint.
    */
   public JointReadOnly getJoint(long jointHashCode)
   {
      if (!hashCodeToJointMap.containsKey(jointHashCode))
         throw new RuntimeException("Unknown joint.");

      return hashCodeToJointMap.get(jointHashCode);
   }

   /**
    * Gets the hash-code associated to the given joint.
    * <p>
    * This is equivalent to {@code joint.hashCode()} excepts that this getter asserts that the joint is
    * already register into this map.
    * </p>
    * 
    * @return the corresponding hash-code.
    * @throws RuntimeException if the joint is not part of this map.
    */
   public long getJointIndex(JointReadOnly joint)
   {
      long hashCode = joint.hashCode();

      if (!hashCodeToJointMap.contains(hashCode))
         throw new RuntimeException("Unknown joint.");

      return hashCode;
   }

   /**
    * Gets all the joints that has been registered to this map.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the collection of all this map's joints.
    */
   public Collection<JointReadOnly> getAllJoints()
   {
      return hashCodeToJointMap.valueCollection();
   }
}
