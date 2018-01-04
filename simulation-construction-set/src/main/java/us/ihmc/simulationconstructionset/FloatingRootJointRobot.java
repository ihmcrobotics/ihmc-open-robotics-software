package us.ihmc.simulationconstructionset;

import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.robotDescription.RobotDescription;

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

   public void setPositionInWorld(Tuple3DReadOnly offset)
   {
      rootJoint.setPosition(offset);
   }

   public void setOrientation(double yaw, double pitch, double roll)
   {
      rootJoint.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setOrientation(QuaternionReadOnly quaternion)
   {
      rootJoint.setQuaternion(quaternion);
   }

   public void setAngularVelocity(Vector3DReadOnly velocity)
   {
      rootJoint.setAngularVelocityInBody(velocity);
   }

   public void setLinearVelocity(Vector3DReadOnly velocity)
   {
      rootJoint.setVelocity(velocity);
   }

   public FloatingJoint getRootJoint()
   {
      return rootJoint;
   }

   public FrameVector3D getRootJointVelocity()
   {
      FrameVector3D ret = new FrameVector3D(ReferenceFrame.getWorldFrame());
      rootJoint.getVelocity(ret.getVector());

      return ret;
   }

   public FrameVector3D getRootJointAngularVelocityInRootJointFrame(ReferenceFrame rootJointFrame)
   {
      Vector3D angularVelocity = rootJoint.getAngularVelocityInBody();
      return new FrameVector3D(rootJointFrame, angularVelocity);
   }

   public Vector3D getPositionInWorld()
   {
      Vector3D position = new Vector3D();
      getPositionInWorld(position);

      return position;
   }

   public void getPositionInWorld(Vector3DBasics vectorToPack)
   {
      rootJoint.getPosition(vectorToPack);
   }

   public void getVelocityInWorld(Vector3DBasics vectorToPack)
   {
      rootJoint.getVelocity(vectorToPack);
   }

   public void getOrientationInWorld(QuaternionBasics quaternionToPack)
   {
      rootJoint.getQuaternion(quaternionToPack);
   }

   public void getAngularVelocityInBody(Vector3DBasics vectorToPack)
   {
      rootJoint.getAngularVelocityInBody(vectorToPack);
   }

}