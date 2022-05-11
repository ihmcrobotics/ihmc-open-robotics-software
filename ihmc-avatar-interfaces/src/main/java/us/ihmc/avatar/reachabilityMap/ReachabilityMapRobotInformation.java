package us.ihmc.avatar.reachabilityMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class ReachabilityMapRobotInformation
{
   private final RobotDefinition robotDefinition;
   private final String baseName;
   private final String endEffectorName;
   private Pose3DReadOnly controlFramePoseInParentJoint;
   private Axis3D orthogonalToPalm = Axis3D.X; // TODO Handle cases where this is not X

   public ReachabilityMapRobotInformation(RobotDefinition robotDefinition, String baseName, String endEffectorName)
   {
      this.robotDefinition = robotDefinition;
      this.baseName = baseName;
      this.endEffectorName = endEffectorName;
   }

   public Axis3D getOrthogonalToPalm()
   {
      return orthogonalToPalm;
   }

   public void setOrthogonalToPalm(Axis3D orthogonalToPalm)
   {
      this.orthogonalToPalm = orthogonalToPalm;
   }

   public Pose3DReadOnly getControlFramePoseInParentJoint()
   {
      return controlFramePoseInParentJoint;
   }

   /**
    * Sets the transform of the end-effector's frame of interest with respect to the end-effector's
    * parent joint frame.
    * <p>
    * It is recommended to align the x-axis of the control frame with the vector that is orthogonal to
    * the palm.
    * </p>
    * 
    * @param controlFramePose the transform from the control frame with respect to the end-effector's
    *                         parent joint frame.
    */
   public void setControlFramePoseInParentJoint(RigidBodyTransformReadOnly controlFramePoseInParentJoint)
   {
      setControlFramePoseInParentJoint(new Pose3D(controlFramePoseInParentJoint));
   }

   /**
    * Sets the transform of the end-effector's frame of interest with respect to the end-effector's
    * parent joint frame.
    * <p>
    * It is recommended to align the x-axis of the control frame with the vector that is orthogonal to
    * the palm.
    * </p>
    * 
    * @param controlFramePose the transform from the control frame with respect to the end-effector's
    *                         parent joint frame.
    */
   public void setControlFramePoseInParentJoint(Pose3DReadOnly controlFramePoseInParentJoint)
   {
      this.controlFramePoseInParentJoint = controlFramePoseInParentJoint;
   }

   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }

   public String getBaseName()
   {
      return baseName;
   }

   public String getEndEffectorName()
   {
      return endEffectorName;
   }

   public List<OneDoFJointDefinition> getEvaluatedJoints()
   {
      return createOneDoFJointPath(robotDefinition, baseName, endEffectorName);
   }

   public static List<OneDoFJointDefinition> createOneDoFJointPath(RobotDefinition robotDefinition, String startRigidBodyName, String endRigidBodyName)
   {
      List<OneDoFJointDefinition> joints = new ArrayList<>();

      RigidBodyDefinition start = robotDefinition.getRigidBodyDefinition(startRigidBodyName);
      RigidBodyDefinition end = robotDefinition.getRigidBodyDefinition(endRigidBodyName);

      // Go from end to start
      RigidBodyDefinition current = end;

      while (current != start)
      {
         JointDefinition parentJoint = current.getParentJoint();

         if (parentJoint == null)
            break;
         if (parentJoint instanceof OneDoFJointDefinition)
            joints.add((OneDoFJointDefinition) parentJoint);

         current = parentJoint.getPredecessor();
      }

      if (current == start)
      { // Success!
         Collections.reverse(joints);
         return joints;
      }
      else
      { // The previous loop failed, we do the same but from start to end
         joints.clear();
         current = start;

         while (current != end)
         {
            JointDefinition parentJoint = current.getParentJoint();

            if (parentJoint == null)
               break;
            if (parentJoint instanceof OneDoFJointDefinition)
               joints.add((OneDoFJointDefinition) parentJoint);

            current = parentJoint.getPredecessor();
         }

         if (current == end)
         { // Success!
            return joints;
         }
         else
         { // We failed again, something's wrong...
            return null;
         }
      }
   }
}
