package us.ihmc.robotModels;

import java.util.Collection;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

/**
 * This class represents a map to retrieve rigid-bodies from their hash-code.
 * <p>
 * Utilities are provided to extract rigid-bodies from {@code FullRobotModel}.
 * </p>
 * <p>
 * Rigid-bodies should be registered at construction and leave this map unchanging at runtime.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class RigidBodyHashCodeResolver
{
   /** Default hash code for any object that is equal to {@code null}. */
   public static final long NULL_HASHCODE = 0L;

   /** The internal map storage. */
   private final TLongObjectHashMap<RigidBodyReadOnly> hashCodeToRigidBodyMap = new TLongObjectHashMap<>();

   /**
    * Creates a new resolver with only the {@code null} rigid-body registered.
    */
   public RigidBodyHashCodeResolver()
   {
      hashCodeToRigidBodyMap.put(NULL_HASHCODE, null);
   }

   /**
    * Creates a new resolver and sets it up to map all the rigid-bodies declared in the given
    * {@code fullRobotModel}.
    * <p>
    * The {@code null} rigid-body is also registered.
    * </p>
    * 
    * @param fullRobotModel the robot model from which rigid-bodies should be registered.
    */
   public RigidBodyHashCodeResolver(FullRobotModel fullRobotModel)
   {
      this();
      putAllFullRobotModelRigidBodies(fullRobotModel);
   }

   /**
    * Registers a new rigid-body into this map.
    * <p>
    * In the case the given {@code rigidBody} was already registered, this map is not modified.
    * </p>
    * 
    * @param rigidBody the rigid-body to register.
    * @throws IllegalArgumentException if a distinct rigid-body was already registered under the same
    *            hash-code as {@code rigidBody.hashCode()}. This usually occurs if: 1- the two
    *            rigid-bodies are two distinct instances with the same name, 2- (very unlikely) bad
    *            luck and the hash-code algorithm generated the same hash-code for the two
    *            rigid-bodies.
    */
   public void put(RigidBodyReadOnly rigidBody)
   {
      put(rigidBody, rigidBody.hashCode());
   }

   /**
    * Registers a new rigid-body into this map with a custom hash-code.
    * <p>
    * In the case the given {@code rigidBody} was already registered with the same hash-code, this map
    * is not modified.
    * </p>
    * 
    * @param rigidBody the rigid-body to register.
    * @param bodyHashCode the custom hash-code to associate with the given rigid-body.
    * @throws IllegalArgumentException if a distinct rigid-body was already registered under the same
    *            hash-code as {@code rigidBody.hashCode()}.
    */
   public void put(RigidBodyReadOnly rigidBody, long bodyHashCode)
   {
      if (hashCodeToRigidBodyMap.containsKey(bodyHashCode))
      {
         RigidBodyReadOnly existingBody = hashCodeToRigidBodyMap.get(bodyHashCode);
         if (rigidBody != existingBody)
         {
            throw new IllegalArgumentException(getClass().getSimpleName()
                  + ": The rigid-body has the same hash-code as another distinct rigid-body previously registered.");
         }
      }
      else
      {
         hashCodeToRigidBodyMap.put(bodyHashCode, rigidBody);
      }
   }

   /**
    * Extracts then registers all the rigid-bodies associated to the given {@code fullRobotModel}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * In the case one of the rigid-bodies of the given {@code fullRobotModel} was already registered,
    * this map is not modified.
    * </p>
    * 
    * @param fullRobotModel the robot model to register the rigid-bodies of.
    * @throws IllegalArgumentException in the case of a hash-code collision, i.e. two distinct
    *            rigid-bodies with the same hash-code.
    */
   public void putAllFullRobotModelRigidBodies(FullRobotModel fullRobotModel)
   {
      putAllMultiBodySystemRigidBodies(fullRobotModel.getElevator());
   }

   /**
    * Extracts then registers all the rigid-bodies associated to the multi-body system attached to the
    * given {@code rootBody}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * In the case one of the rigid-bodies of the multi-body system was already registered, this map is
    * not modified.
    * </p>
    * 
    * @param rootBody the root of the multi-body system to register the rigid-bodies of.
    * @throws IllegalArgumentException in the case of a hash-code collision, i.e. two distinct
    *            rigid-bodies with the same hash-code.
    */
   public void putAllMultiBodySystemRigidBodies(RigidBodyReadOnly rootBody)
   {
      putAll(rootBody.subtreeList());
   }

   /**
    * Registers a collection of new rigid-bodies into this map.
    * <p>
    * In the case a rigid-body of {@code rigidBodies} was already registered, this map is not modified.
    * </p>
    * 
    * @param rigidBodies the new rigid-bodies to register.
    * @throws IllegalArgumentException in the case of a hash-code collision, i.e. two distinct
    *            rigid-bodies with the same hash-code.
    */
   public void putAll(Collection<? extends RigidBodyReadOnly> rigidBodies)
   {
      rigidBodies.forEach(this::put);
   }

   /**
    * Navigates the given map and registers the different rigid-bodies with a given custom ID.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param customIdsToRigidBodyMap the map from custom ID to rigid-body.
    * @throws IllegalArgumentException if a distinct rigid-body was already registered under the same
    *            hash-code.
    */
   public void registerCustomRigidBodyIds(TLongObjectMap<? extends RigidBodyReadOnly> customIdsToRigidBodyMap)
   {
      if (customIdsToRigidBodyMap == null)
         return;

      for (long key : customIdsToRigidBodyMap.keys())
      {
         RigidBodyReadOnly rigidBody = customIdsToRigidBodyMap.get(key);

         if (rigidBody != null)
         {
            put(rigidBody);
            put(rigidBody, key);
         }
      }
   }

   /**
    * Casts then gets the rigid-body associated to the given hash-code.
    * 
    * @param bodyHashCode the hash-code used to retrieve the rigid-body, it is usually generated from
    *           {@code rigidBody.hashCode()}.
    * @return the corresponding rigid-body casted.
    * @param <R> the type the rigid-body is casted to.
    */
   @SuppressWarnings("unchecked")
   public <R extends RigidBodyReadOnly> R castAndGetRigidBody(long bodyHashCode)
   {
      return (R) getRigidBody(bodyHashCode);
   }

   /**
    * Gets the rigid-body associated to the given hash-code.
    * 
    * @param bodyHashCode the hash-code used to retrieve the rigid-body, it is usually generated from
    *           {@code rigidBody.hashCode()}.
    * @return the corresponding rigid-body.
    */
   public RigidBodyReadOnly getRigidBody(long bodyHashCode)
   {
      if (!hashCodeToRigidBodyMap.containsKey(bodyHashCode))
         throw new RuntimeException("Unknown rigid-body.");

      return hashCodeToRigidBodyMap.get(bodyHashCode);
   }

   /**
    * Gets the hash-code associated to the given rigid-body.
    * <p>
    * This is equivalent to {@code rigidBody.hashCode()} excepts that this getter asserts that the
    * rigid-body is already register into this map.
    * </p>
    * 
    * @return the corresponding hash-code.
    * @throws RuntimeException if the rigid-body is not part of this map.
    */
   public long getRigidBodyIndex(RigidBodyReadOnly rigidBody)
   {
      long hashCode = rigidBody.hashCode();

      if (!hashCodeToRigidBodyMap.contains(hashCode))
         throw new RuntimeException("Unknown rigid-body.");

      return hashCode;
   }

   /**
    * Gets all the rigid-bodies that has been registered to this map.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the collection of all this map's rigid-bodies.
    */
   public Collection<RigidBodyReadOnly> getAllRigidBodies()
   {
      return hashCodeToRigidBodyMap.valueCollection();
   }
}
