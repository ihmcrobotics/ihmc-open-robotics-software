package us.ihmc.robotModels;

import java.util.Collection;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

public class JointHashCodeResolver
{
   /** Default hash code for any object that is equal to {@code null}. */
   public static final long NULL_HASHCODE = 0L;

   /** the internal map storage. */
   private final TLongObjectHashMap<JointReadOnly> hashCodeToJointMap = new TLongObjectHashMap<>();

   public JointHashCodeResolver()
   {
      hashCodeToJointMap.put(NULL_HASHCODE, null);
   }

   public JointHashCodeResolver(FullRobotModel fullRobotModel)
   {
      this();
      putAllFullRobotModelJoints(fullRobotModel);
   }

   public void put(JointReadOnly joint)
   {
      put(joint, joint.hashCode());
   }

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

   public void putAllFullRobotModelJoints(FullRobotModel fullRobotModel)
   {
      putAllMultiBodySystemJoints(fullRobotModel.getElevator());
   }

   public void putAllMultiBodySystemJoints(RigidBodyReadOnly rootBody)
   {
      rootBody.childrenSubtreeIterable().forEach(this::put);
   }

   public void putAllMultiBodySystemJoints(JointReadOnly rootJoint)
   {
      putAll(rootJoint.subtreeList());
   }

   public void putAll(Collection<? extends JointReadOnly> joints)
   {
      joints.forEach(this::put);
   }

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

   @SuppressWarnings("unchecked")
   public <J extends JointReadOnly> J castAndGetJoint(long jointHashCode)
   {
      return (J) getJoint(jointHashCode);
   }

   public JointReadOnly getJoint(long bodyHashCode)
   {
      if (!hashCodeToJointMap.containsKey(bodyHashCode))
         throw new RuntimeException("Unknown rigid-body.");

      return hashCodeToJointMap.get(bodyHashCode);
   }

   public long getJointIndex(JointReadOnly joint)
   {
      long hashCode = joint.hashCode();

      if (!hashCodeToJointMap.contains(hashCode))
         throw new RuntimeException("Unknown joint.");

      return hashCode;
   }

   public Collection<JointReadOnly> getAllJoints()
   {
      return hashCodeToJointMap.valueCollection();
   }
}
