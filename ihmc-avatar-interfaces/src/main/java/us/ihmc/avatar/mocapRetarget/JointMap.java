package us.ihmc.avatar.mocapRetarget;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.SkeletonHierarchy;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.JointInfo;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import java.util.HashMap;
import java.util.Map;

/**
 * Bridges BVH outputs to the robot control stack.
 *
 * Supports:
 *  A) BVH joint → robot OneDoF joint angle extraction (returns Map<OneDoFJointBasics, Double>).
 *  B) BVH joint → robot rigid body end-effector pose messages for the Kinematics Streaming Toolbox.
 *
 * Assumptions:
 *  - CoordinateTransformer outputs GLOBAL BVH transforms per joint.
 *  - Angles are radians.
 *  - If needed, a BVH-to-world alignment can be provided (default = identity).
 */
public class JointMap
{
   private final SkeletonHierarchy skeletonHierarchy;
   private final FullHumanoidRobotModel robotModel;

   /** BVH joint name → OneDoF joint (for joint-angle pipelines / debugging). */
   private final Map<String, OneDoFJointBasics> bvhToRobotJoint = new HashMap<>();

   /** BVH joint name → RigidBody (for IK streaming via rigid-body messages). */
   private final Map<String, RigidBodyBasics> bvhToEndEffector = new HashMap<>();

   /** Optional alignment from BVH globals to robot WORLD frame. Defaults to identity. */
   private final RigidBodyTransform bvhToWorld = new RigidBodyTransform();

   /** Scratch to avoid allocations. */
   private final RigidBodyTransform scratchWorldPose = new RigidBodyTransform();
   private final RigidBodyTransform scratchRel = new RigidBodyTransform();
   private final Vector3D scratchRotVec = new Vector3D();
   private final Vector3D scratchAxis = new Vector3D();
   private final Quaternion scratchQuat = new Quaternion();

   /** BVHtoWorld Robot Alignment*/
   private double positionScale = 1.0; // optional uniform scaling for positions


   public JointMap(SkeletonHierarchy skeletonHierarchy, FullHumanoidRobotModel robotModel)
   {
      this.skeletonHierarchy = skeletonHierarchy;
      this.robotModel = robotModel;
      this.bvhToWorld.setIdentity();
   }

   /** Map a BVH joint to a robot OneDoF joint (for angle extraction). */
   public void addJointAngleMapping(String bvhJointName, OneDoFJointBasics robotJoint)
   {
      bvhToRobotJoint.put(bvhJointName, robotJoint);
   }

   /** Map a BVH joint to a robot rigid body (for task-space IK streaming). */
   public void addEndEffectorMapping(String bvhJointName, RigidBodyBasics rigidBody)
   {
      bvhToEndEffector.put(bvhJointName, rigidBody);
   }

   /** Optional: set BVH→robot world alignment (e.g., scale/rotate/translate). */
   public void setBvhToWorld(RigidBodyTransform alignment)
   {
      this.bvhToWorld.set(alignment);
   }

   /**
    * Extract desired joint angles (radians) for mapped OneDoF joints, using BVH parent→child relative rotation.
    * Returns a sparse map for the joints you mapped.
    *
    * Note: this simple projection uses the rotation-vector dot the joint axis in the parent frame.
    * Depending on your robot’s joint axis definition, you may need to express the axis in the same frame as 'relative'.
    */
   public Map<OneDoFJointBasics, Double> mapFrameToJointAngles(Map<String, RigidBodyTransform> bvhGlobals)
   {
      Map<OneDoFJointBasics, Double> desiredAngles = new HashMap<>();

      for (Map.Entry<String, OneDoFJointBasics> e : bvhToRobotJoint.entrySet())
      {
         String bvhJoint = e.getKey();
         OneDoFJointBasics joint = e.getValue();

         JointInfo ji = skeletonHierarchy.getJoint(bvhJoint);
         if (ji == null) continue;

         String parentName = ji.parentName();
         if (parentName == null || parentName.isEmpty()) continue; // skip root

         RigidBodyTransform parentGlobal = bvhGlobals.get(parentName);
         RigidBodyTransform childGlobal  = bvhGlobals.get(bvhJoint);
         if (parentGlobal == null || childGlobal == null) continue;

         // relative = parent^-1 * child in BVH frame
         scratchRel.setAndInvert(parentGlobal);
         scratchRel.multiply(childGlobal);

         // rotation vector (axis * angle) in that parent frame
         scratchRel.getRotation().getRotationVector(scratchRotVec);

         // Joint axis (assumed expressed in the same frame as 'relative' → parent frame).
         scratchAxis.set(joint.getJointAxis()); // if needed, transform this axis into the same frame

         double projectedAngle = scratchAxis.dot(scratchRotVec); // radians
         desiredAngles.put(joint, projectedAngle);
      }

      return desiredAngles;
   }

   public void setPositionScale(double scale)
   {
      this.positionScale = scale;
   }
   /**
    * Calibrate BVH→WORLD using the pelvis:
    *   bvhToWorld = desiredPelvisWorldPose * inverse(bvhPelvisGlobal)
    *
    * @param bvhPelvisName      BVH joint name for pelvis/root (e.g., "Hips")
    * @param desiredPelvisWorld target pelvis pose in WORLD (e.g., robot pelvis world pose or origin)
    * @param bvhGlobals         BVH global transforms for a calibration frame (usually frame 0)
    * @param alignYawOnly       if true, only align yaw (Z-rotation); ignore roll/pitch from both sides
    */
   public void calibrateBvhToWorldFromPelvis(String bvhPelvisName,
                                             RigidBodyTransform desiredPelvisWorld,
                                             Map<String, RigidBodyTransform> bvhGlobals,
                                             boolean alignYawOnly)
   {
      RigidBodyTransform bvhPelvis = bvhGlobals.get(bvhPelvisName);
      if (bvhPelvis == null)
         throw new IllegalArgumentException("BVH pelvis joint not found in globals: " + bvhPelvisName);

      if (!alignYawOnly)
      {
         // Full 6-DoF alignment
         RigidBodyTransform inv = new RigidBodyTransform(bvhPelvis);
         inv.invert();
         bvhToWorld.set(desiredPelvisWorld);
         bvhToWorld.multiply(inv);
         return;
      }

      // Create temp object to store results
      YawPitchRoll ypr = new YawPitchRoll();

      // BVH pelvis yaw/pitch/roll
      YawPitchRollConversion.convertMatrixToYawPitchRoll(bvhPelvis.getRotation(), ypr);
      double bvhYaw = ypr.getYaw();

      // Desired pelvis yaw/pitch/roll
      YawPitchRollConversion.convertMatrixToYawPitchRoll(desiredPelvisWorld.getRotation(), ypr);
      double desYaw = ypr.getYaw();


      // 2) Build yaw-only rotations
      us.ihmc.euclid.matrix.RotationMatrix R_bvhYaw = new us.ihmc.euclid.matrix.RotationMatrix();
      R_bvhYaw.setToYawOrientation(bvhYaw);

      us.ihmc.euclid.matrix.RotationMatrix R_desYaw = new us.ihmc.euclid.matrix.RotationMatrix();
      R_desYaw.setToYawOrientation(desYaw);

      // 3) Build yaw-only pelvis transforms (keep original positions)
      RigidBodyTransform T_bvhYaw = new RigidBodyTransform();
      T_bvhYaw.getRotation().set(R_bvhYaw);
      T_bvhYaw.getTranslation().set(bvhPelvis.getTranslation());

      RigidBodyTransform T_desYaw = new RigidBodyTransform();
      T_desYaw.getRotation().set(R_desYaw);
      T_desYaw.getTranslation().set(desiredPelvisWorld.getTranslation());

      // 4) bvhToWorld = T_desYaw * inv(T_bvhYaw)
      RigidBodyTransform invYaw = new RigidBodyTransform(T_bvhYaw);
      invYaw.invert();
      bvhToWorld.set(T_desYaw);
      bvhToWorld.multiply(invYaw);
   }


   /**
    * Build a KinematicsStreamingToolboxInputMessage by creating a RigidBody message
    * for each mapped end-effector. Desired poses are taken from BVH globals and expressed in WORLD frame as required by the toolbox.
    */
   public KinematicsStreamingToolboxInputMessage toStreamingMessage(Map<String, RigidBodyTransform> bvhGlobals,
                                                                    long sequenceId,
                                                                    long timestampNanos,
                                                                    boolean streamToController)
   {
      KinematicsStreamingToolboxInputMessage msg = new KinematicsStreamingToolboxInputMessage();
      msg.setSequenceId(sequenceId);
      msg.setTimestamp(timestampNanos);
      msg.setStreamToController(streamToController);

      for (Map.Entry<String, RigidBodyBasics> e : bvhToEndEffector.entrySet())
      {
         String bvhJoint = e.getKey();
         RigidBodyBasics body = e.getValue();

         RigidBodyTransform bvhPose = bvhGlobals.get(bvhJoint);
         if (bvhPose == null) continue;

         // worldPose = bvhToWorld * bvhPose
         scratchWorldPose.set(bvhToWorld);
         scratchWorldPose.multiply(bvhPose);

         // --- Apply uniform scale to translation (RigidBodyTransform can't store scale)
         double sx = scratchWorldPose.getTranslation().getX() * positionScale;
         double sy = scratchWorldPose.getTranslation().getY() * positionScale;
         double sz = scratchWorldPose.getTranslation().getZ() * positionScale;

         KinematicsToolboxRigidBodyMessage input = msg.getInputs().add();
         input.setEndEffectorHashCode(body.hashCode());

         input.getDesiredPositionInWorld().set(sx, sy, sz);

         scratchQuat.set(scratchWorldPose.getRotation());
         input.getDesiredOrientationInWorld().set(scratchQuat);

         // Optional: selection matrices/rate limits can be set here
      }

      return msg;
   }

}
