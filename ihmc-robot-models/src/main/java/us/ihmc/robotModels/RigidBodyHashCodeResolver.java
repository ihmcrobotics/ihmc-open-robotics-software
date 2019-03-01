package us.ihmc.robotModels;

import java.util.Collection;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

public class RigidBodyHashCodeResolver
{
   /** Default hash code for any object that is equal to {@code null}. */
   public static final long NULL_HASHCODE = 0L;

   /** the internal map storage. */
   private final TLongObjectHashMap<RigidBodyReadOnly> hashCodeToRigidBodyMap = new TLongObjectHashMap<>();

   public RigidBodyHashCodeResolver()
   {
      hashCodeToRigidBodyMap.put(NULL_HASHCODE, null);
   }

   public RigidBodyHashCodeResolver(FullRobotModel fullRobotModel)
   {
      this();
      putAllFullRobotModelRigidBodies(fullRobotModel);
   }

   public void put(RigidBodyReadOnly rigidBody)
   {
      put(rigidBody, rigidBody.hashCode());
   }

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

   public void putAllFullRobotModelRigidBodies(FullRobotModel fullRobotModel)
   {
      putAllMultiBodySystemRigidBodies(fullRobotModel.getElevator());
   }

   public void putAllMultiBodySystemRigidBodies(RigidBodyReadOnly rootBody)
   {
      putAll(rootBody.subtreeList());
   }

   public void putAll(Collection<? extends RigidBodyReadOnly> rigidBodies)
   {
      rigidBodies.forEach(this::put);
   }

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

   @SuppressWarnings("unchecked")
   public <R extends RigidBodyReadOnly> R castAndGetRigidBody(long bodyHashCode)
   {
      return (R) getRigidBody(bodyHashCode);
   }

   public RigidBodyReadOnly getRigidBody(long bodyHashCode)
   {
      if (!hashCodeToRigidBodyMap.containsKey(bodyHashCode))
         throw new RuntimeException("Unknown rigid-body.");

      return hashCodeToRigidBodyMap.get(bodyHashCode);
   }

   public long getRigidBodyIndex(RigidBodyReadOnly rigidBody)
   {
      long hashCode = rigidBody.hashCode();

      if (!hashCodeToRigidBodyMap.contains(hashCode))
         throw new RuntimeException("Unknown rigid-body.");

      return hashCode;
   }

   public Collection<RigidBodyReadOnly> getAllRigidBodies()
   {
      return hashCodeToRigidBodyMap.valueCollection();
   }
}
