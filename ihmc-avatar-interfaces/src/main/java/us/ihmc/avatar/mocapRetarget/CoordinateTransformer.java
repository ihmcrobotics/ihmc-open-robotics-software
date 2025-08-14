package us.ihmc.avatar.mocapRetarget;

import us.ihmc.avatar.mocapRetarget.bvh.MotionFrame;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.SkeletonHierarchy;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.JointInfo;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.*;

/**
 * Builds per-frame GLOBAL transforms for all joints using Convention A:
 *   local(j) = R(channels in listed order)  [root also has T(rootPosition)]
 *   global(root) = local(root)
 *   global(child) = global(parent) * T(OFFSET_child) * local(child)
 */
public class CoordinateTransformer
{
   /** Static skeleton info. */
   private final SkeletonHierarchy skeleton;

   /** Stable joint index order (preorder) for FK; index 0 is root. */
   private final List<JointInfo> jointsPreorder;
   /** Name -> index into preorder lists. */
   private final Map<String, Integer> nameToIndex;

   /** Children adjacency, by joint name. */
   private final Map<String, List<JointInfo>> childrenByName;

   /** Precomputed T(OFFSET) per joint index. */
   private final RigidBodyTransform[] offsetTransforms;

   /** Reusable output buffer per frame (globals). */
   private final RigidBodyTransform[] globalsBuffer;

   /** Reusable scratch (avoid allocations in inner loop). */
   private final RotationMatrix scratchRot = new RotationMatrix();
   private final RigidBodyTransform scratchStep = new RigidBodyTransform();

   public CoordinateTransformer(SkeletonHierarchy skeleton)
   {
      this.skeleton = skeleton;

      // --- Build children lists & find root(s)
      this.childrenByName = buildChildrenAdjacency(skeleton);
      JointInfo root = findRoot(skeleton);

      // --- Build a deterministic preorder traversal starting at root
      this.jointsPreorder = new ArrayList<>();
      preorderCollect(root, jointsPreorder, childrenByName);

      // --- Build name->index map for quick lookup
      this.nameToIndex = new HashMap<>();
      for (int i = 0; i < jointsPreorder.size(); i++)
         nameToIndex.put(jointsPreorder.get(i).name(), i);

      // --- Precompute T(OFFSET) per joint index (Convention A)
      this.offsetTransforms = new RigidBodyTransform[jointsPreorder.size()];
      for (int i = 0; i < jointsPreorder.size(); i++)
      {
         offsetTransforms[i] = new RigidBodyTransform();
         offsetTransforms[i].setIdentity();

         Vector3D off = jointsPreorder.get(i).offset(); // <-- ensure JointInfo.offset() returns a Vector3D
         if (off != null)
            offsetTransforms[i].getTranslation().set(off);
      }

      // --- Allocate a reusable globals buffer (one per joint)
      this.globalsBuffer = new RigidBodyTransform[jointsPreorder.size()];
      for (int i = 0; i < globalsBuffer.length; i++)
         globalsBuffer[i] = new RigidBodyTransform();
   }

   /**
    * Compute GLOBAL transforms for all joints for a single frame.
    * Returns a Map view (name->RigidBodyTransform). The transforms are reused each call;
    * copy them if you need to keep them.
    */
   public Map<String, RigidBodyTransform> buildGlobalTransforms(MotionFrame frame)
   {
      // 1) Build locals & FK in preorder
      for (int idx = 0; idx < jointsPreorder.size(); idx++)
      {
         JointInfo joint = jointsPreorder.get(idx);
         boolean isRoot = isRoot(joint);

         // local(joint): rotation from channels (and root translation if present)
         RigidBodyTransform local = buildLocalTransform(frame, joint);

         if (isRoot)
         {
            // global(root) = local(root)
            globalsBuffer[idx].set(local);
         }
         else
         {
            // global(child) = global(parent) * T(OFFSET_child) * local(child)
            int parentIdx = nameToIndex.get(joint.parentName());
            RigidBodyTransform parentGlobal = globalsBuffer[parentIdx];

            // scratchStep = T(OFFSET_child) * local(child)
            scratchStep.set(offsetTransforms[idx]);
            // append local rotation (and translation if any, though non-root should have none)
            scratchStep.getRotation().multiply(local.getRotation());
            scratchStep.getTranslation().add(local.getTranslation()); // typically zero for non-root

            // globalsBuffer[idx] = parentGlobal * scratchStep
            globalsBuffer[idx].set(parentGlobal);
            globalsBuffer[idx].multiply(scratchStep);
         }
      }

      // 2) Expose a Map<String, RigidBodyTransform> view over globalsBuffer.
      //    (No copies; if you want copies, materialize them here.)
      Map<String, RigidBodyTransform> out = new LinkedHashMap<>();
      for (int i = 0; i < jointsPreorder.size(); i++)
         out.put(jointsPreorder.get(i).name(), globalsBuffer[i]);

      return out;
   }

   /**
    * Builds a LOCAL transform for a single joint for the given frame:
    *   - rotation composed in the BVH per-joint CHANNELS order (degrees→radians).
    *   - translation set ONLY for the root from X/Y/Zposition (Convention A).
    *   - DOES NOT include the static OFFSET (that is applied in FK).
    */
   public RigidBodyTransform buildLocalTransform(MotionFrame frame, JointInfo joint)
   {
      RigidBodyTransform local = new RigidBodyTransform();
      local.setIdentity();

      int start = joint.channelStartIndex();   // <-- ensure your JointInfo exposes this
      int count = joint.channelCount();
      List<String> channels = joint.channels(); // <-- ordered channel names

      // Compose rotation in EXACT file order
      RotationMatrix R = (RotationMatrix) local.getRotation();
      R.setIdentity();

      boolean isRoot = isRoot(joint);
      Vector3D T = (Vector3D) local.getTranslation();
      if (isRoot) T.setToZero(); // root translation may be set by position channels; start at zero
      else T.setToZero();        // non-root must be zero translation

      for (int k = 0; k < count; k++)
      {
         String channelName = channels.get(k);
         double value = frame.channelData()[start + k]; // flat per-frame channel array

         // Rotation channels (degrees → radians)
         if (channelName.endsWith("rotation"))
         {
            double rad = Math.toRadians(value);
            if (channelName.startsWith("X"))
               R.appendRollRotation(rad);   // X rotation (roll)
            else if (channelName.startsWith("Y"))
               R.appendPitchRotation(rad);  // Y rotation (pitch)
            else if (channelName.startsWith("Z"))
               R.appendYawRotation(rad);    // Z rotation (yaw)
         }
         // Position channels (root only)
         else if (channelName.endsWith("position") && isRoot)
         {
            if (channelName.startsWith("X"))
               T.setX(value);
            else if (channelName.startsWith("Y"))
               T.setY(value);
            else if (channelName.startsWith("Z"))
               T.setZ(value);
         }
      }

      return local;
   }

   // ===========================
   // ----- helper methods ------
   // ===========================

   private static boolean isRoot(JointInfo j)
   {
      // Adjust if your JointInfo exposes a different root marker
      return j.parentName() == null || j.parentName().isEmpty();
   }

   private static JointInfo findRoot(SkeletonHierarchy skeleton)
   {
      // If your SkeletonHierarchy exposes getRoot(), use that instead.
      for (JointInfo j : skeleton.joints.values()) // relies on your map of joints
      {
         if (j.parentName() == null || j.parentName().isEmpty())
            return j;
      }
      throw new IllegalStateException("No root joint found.");
   }

   private static Map<String, List<JointInfo>> buildChildrenAdjacency(SkeletonHierarchy skeleton)
   {
      Map<String, List<JointInfo>> children = new HashMap<>();
      // ensure each joint key exists
      for (JointInfo j : skeleton.joints.values())
         children.computeIfAbsent(j.name(), k -> new ArrayList<>());

      for (JointInfo j : skeleton.joints.values())
      {
         String parent = j.parentName();
         if (parent != null && !parent.isEmpty())
            children.computeIfAbsent(parent, k -> new ArrayList<>()).add(j);
      }
      return children;
   }

   private static void preorderCollect(JointInfo root,
                                       List<JointInfo> out,
                                       Map<String, List<JointInfo>> childrenByName)
   {
      Deque<JointInfo> stack = new ArrayDeque<>();
      stack.push(root);
      while (!stack.isEmpty())
      {
         JointInfo current = stack.pop();
         out.add(current);

         List<JointInfo> kids = childrenByName.getOrDefault(current.name(), Collections.emptyList());
         // push in reverse to visit in insertion order
         ListIterator<JointInfo> it = kids.listIterator(kids.size());
         while (it.hasPrevious())
            stack.push(it.previous());
      }
   }
}




//package us.ihmc.avatar.mocapRetarget;
//
//import us.ihmc.avatar.mocapRetarget.bvh.MotionFrame;
//import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.JointInfo;
//import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.SkeletonHierarchy;
//import us.ihmc.euclid.transform.RigidBodyTransform;
//import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseKinematicsSolver;
//import us.ihmc.communication.packetCommunicator.PacketCommunicator;
//
//import java.rmi.server.Skeleton;
//import java.util.LinkedHashMap;
//import java.util.Map;
//
//public class CoordinateTransformer
//{
//   private SkeletonHierarchy skeleton;
//   private Map<String, RigidBodyTransform> frameTransforms = new LinkedHashMap<>();
//
//   public CoordinateTransformer(SkeletonHierarchy skeleton) {
//      this.skeleton = skeleton;
//   }
//
//   public Map<String, RigidBodyTransform> buildGlobalTransforms(MotionFrame frame) {
//      return frameTransforms;
//   }
//
//   public RigidBodyTransform buildLocalTransform(MotionFrame frame, JointInfo joint) {
//     RigidBodyTransform localTransform = new RigidBodyTransform();
//     localTransform.setIdentity();
//
//      for (int i = joint.channelStartIndex(); i < (joint.channelStartIndex() + joint.channelCount()); i++) {
//         String channelName = joint.channels().get(i - joint.channelStartIndex());
//         if (channelName.endsWith("rotation")) {
//            double radAngle = Math.toRadians(frame.channelData()[i]);
//            if (channelName.startsWith("X")) {
//               localTransform.getRotation().appendRollRotation(radAngle);
//            } else if (channelName.startsWith("Y")) {
//               localTransform.getRotation().appendPitchRotation(radAngle);
//            } else if (channelName.startsWith("Z")) {
//               localTransform.getRotation().appendYawRotation(radAngle);
//            }
//         } else if (channelName.endsWith("position")) {
//            double translation = frame.channelData()[i];
//            if (channelName.startsWith("X")) {
//               localTransform.getTranslation().setX(translation);
//            } else if (channelName.startsWith("Y")) {
//               localTransform.getTranslation().setY(translation);
//            } else if (channelName.startsWith("Z")) {
//               localTransform.getTranslation().setZ(translation);
//            }
//         }
//      }
//     return localTransform;
//   }
//
//
//}
