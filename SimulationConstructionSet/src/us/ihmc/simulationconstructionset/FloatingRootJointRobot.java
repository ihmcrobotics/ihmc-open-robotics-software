package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.tools.FormattingTools;

public class FloatingRootJointRobot extends RobotFromDescription
{
   private static final boolean DEBUG = false;
   private final FloatingJoint rootJoint;

   public FloatingRootJointRobot(RobotDescription robotDescription)
   {
      this(robotDescription, true, true);
   }
   
   public FloatingRootJointRobot(RobotDescription robotDescription, boolean enableDamping, boolean enableJointTorqueAndVelocityLimits)
   {
      super(robotDescription, enableDamping, enableJointTorqueAndVelocityLimits);

      rootJoint = (FloatingJoint) this.getRootJoints().get(0);

      Point3D centerOfMass = new Point3D();
      double totalMass = computeCenterOfMass(centerOfMass);
      if (DEBUG)
         System.out.println("SDFRobot: Total robot mass: " + FormattingTools.getFormattedDecimal3D(totalMass) + " (kg)");
   }

   public Quaternion getRootJointToWorldRotationQuaternion()
   {
      return rootJoint.getQuaternion();
   }

   public void getRootJointToWorldTransform(RigidBodyTransform transform)
   {
      rootJoint.getTransformToWorld(transform);
   }

   public void setPositionInWorld(Tuple3DBasics offset)
   {
      rootJoint.setPosition(offset);
   }

   public void setOrientation(double yaw, double pitch, double roll)
   {
      rootJoint.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setOrientation(Quaternion quaternion)
   {
      rootJoint.setQuaternion(quaternion);
   }

   public void setAngularVelocity(Vector3D velocity)
   {
      rootJoint.setAngularVelocityInBody(velocity);
   }

   public void setLinearVelocity(Vector3D velocity)
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

   public FrameVector getRootJointAngularVelocityInRootJointFrame(ReferenceFrame rootJointFrame)
   {
      Vector3D angularVelocity = rootJoint.getAngularVelocityInBody();
      return new FrameVector(rootJointFrame, angularVelocity);
   }

   public Vector3D getPositionInWorld()
   {
      Vector3D position = new Vector3D();
      getPositionInWorld(position);

      return position;
   }

   public void getPositionInWorld(Vector3D vectorToPack)
   {
      rootJoint.getPosition(vectorToPack);
   }

   public void getVelocityInWorld(Vector3D vectorToPack)
   {
      rootJoint.getVelocity(vectorToPack);
   }

   public void getOrientationInWorld(Quaternion quaternionToPack)
   {
      rootJoint.getQuaternion(quaternionToPack);
   }

   public void getAngularVelocityInBody(Vector3D vectorToPack)
   {
      rootJoint.getAngularVelocityInBody(vectorToPack);
   }

}