package us.ihmc.SdfLoader;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.tools.FormattingTools;

public class SDFRobot extends RobotFromDescription
{
   private static final boolean DEBUG = false;
   private final FloatingJoint rootJoint;

   public SDFRobot(RobotDescription robotDescription)
   {
      super(robotDescription);

      rootJoint = (FloatingJoint) this.getRootJoints().get(0);

      Point3d centerOfMass = new Point3d();
      double totalMass = computeCenterOfMass(centerOfMass);
      if (DEBUG) System.out.println("SDFRobot: Total robot mass: " + FormattingTools.getFormattedDecimal3D(totalMass) + " (kg)");
   }

   public Quat4d getRootJointToWorldRotationQuaternion()
   {
      return rootJoint.getQuaternion();
   }

   public void getRootJointToWorldTransform(RigidBodyTransform transform)
   {
      rootJoint.getTransformToWorld(transform);
   }

   public void setPositionInWorld(Tuple3d offset)
   {
      rootJoint.setPosition(offset);
   }

   public void setOrientation(double yaw, double pitch, double roll)
   {
      rootJoint.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setOrientation(Quat4d quaternion)
   {
      rootJoint.setQuaternion(quaternion);
   }

   public void setAngularVelocity(Vector3d velocity)
   {
      rootJoint.setAngularVelocityInBody(velocity);
   }

   public void setLinearVelocity(Vector3d velocity)
   {
      rootJoint.setVelocity(velocity);
   }

   public FloatingJoint getRootJoint()
   {
      return rootJoint;
   }


   public FrameVector getRootJointVelocity()
   {
      FrameVector ret = new FrameVector(ReferenceFrame.getWorldFrame());
      rootJoint.getVelocity(ret.getVector());

      return ret;
   }

   public FrameVector getPelvisAngularVelocityInPelvisFrame(ReferenceFrame pelvisFrame)
   {
      Vector3d angularVelocity = rootJoint.getAngularVelocityInBody();

      return new FrameVector(pelvisFrame, angularVelocity);
   }

   public Vector3d getPositionInWorld()
   {
      Vector3d position = new Vector3d();
      getPositionInWorld(position);

      return position;
   }

   public void getPositionInWorld(Vector3d vectorToPack)
   {
      rootJoint.getPosition(vectorToPack);
   }

   public void getVelocityInWorld(Vector3d vectorToPack)
   {
      rootJoint.getVelocity(vectorToPack);
   }

   public void getOrientationInWorld(Quat4d quaternionToPack)
   {
      rootJoint.getQuaternion(quaternionToPack);
   }

   public void getAngularVelocityInBody(Vector3d vectorToPack)
   {
      rootJoint.getAngularVelocityInBody(vectorToPack);
   }

   public Joint getJoint(String jointName)
   {
      return getJointRecursively(this.getRootJoints(), jointName);
   }

   private Joint getJointRecursively(ArrayList<Joint> joints, String jointName)
   {
      for (Joint joint : joints)
      {
         String nextJointName = joint.getName();

         if (nextJointName.equals(jointName))
         {
            return joint;
         }

         ArrayList<Joint> children = joint.getChildrenJoints();

         Joint foundJoint = getJointRecursively(children, jointName);
         if (foundJoint != null)
            return foundJoint;
      }

      return null;
   }

   public FloatingJoint getPelvisJoint()
   {
      return getRootJoint();
   }


}