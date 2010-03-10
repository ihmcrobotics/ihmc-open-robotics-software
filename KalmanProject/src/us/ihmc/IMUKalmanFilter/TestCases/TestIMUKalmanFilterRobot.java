package us.ihmc.IMUKalmanFilter.TestCases;

import com.yobotics.simulationconstructionset.*;
import javax.vecmath.*;
import javax.media.j3d.Transform3D;

public class TestIMUKalmanFilterRobot extends Robot
{
   // Primary joints of the robot...
   public FloatingJoint rootJoint;

   public final YoVariable q_x, q_y, q_z;    // Position in world coordinates
   public final YoVariable qd_x, qd_y, qd_z;    // Velocity in world coordinates.
   public final YoVariable qdd_x, qdd_y, qdd_z;    // Acceleration in world coordinates.

   public final YoVariable q_qs, q_qx, q_qy, q_qz;    // Rotation as a quaternion.
   public final YoVariable qd_wx, qd_wy, qd_wz;    // Angular velocities in world coordinates
   public final YoVariable qdd_wx, qdd_wy, qdd_wz;    // Angular acceleration in world coordinates

   // Rotation of the body
   private final YoVariableRegistry registry = this.getRobotsYoVariableRegistry();
   public final YoVariable yaw = new YoVariable("yaw", registry);
   public final YoVariable pitch = new YoVariable("pitch", registry);
   public final YoVariable roll = new YoVariable("roll", registry);

   public final YoVariable ef_body_fx, ef_body_fy, ef_body_fz;    // External Forces in world coordinates

   public TestIMUKalmanFilterRobot()
   {
      super("TestIMU");

      rootJoint = new FloatingJoint("root", new Vector3d(0.0, 0.0, 0.0), this);

      Link link = new Link("ahrs");
      link.addCoordinateSystem(0.2);
      link.translate(0.0, 0.0, -0.05);
      link.addCube(0.1, 0.1, 0.1);

      link.setMass(1.0);
      link.setMomentOfInertia(1.0, 1.0, 1.0);

      ExternalForcePoint ef_body = new ExternalForcePoint("ef_body", new Vector3d(), this);
      rootJoint.addExternalForcePoint(ef_body);

      rootJoint.setLink(link);
      addRootJoint(rootJoint);

      q_x = this.getVariable("q_x");
      q_y = this.getVariable("q_y");
      q_z = this.getVariable("q_z");

      qd_x = this.getVariable("qd_x");
      qd_y = this.getVariable("qd_y");
      qd_z = this.getVariable("qd_z");

      qdd_x = this.getVariable("qdd_x");
      qdd_y = this.getVariable("qdd_y");
      qdd_z = this.getVariable("qdd_z");

      q_qs = this.getVariable("q_qs");
      q_qx = this.getVariable("q_qx");
      q_qy = this.getVariable("q_qy");
      q_qz = this.getVariable("q_qz");

      qd_wx = this.getVariable("qd_wx");
      qd_wy = this.getVariable("qd_wy");
      qd_wz = this.getVariable("qd_wz");

      qdd_wx = this.getVariable("qdd_wx");
      qdd_wy = this.getVariable("qdd_wy");
      qdd_wz = this.getVariable("qdd_wz");

      ef_body_fx = this.getVariable("ef_body_fx");
      ef_body_fy = this.getVariable("ef_body_fy");
      ef_body_fz = this.getVariable("ef_body_fz");

      this.addYoVariableRegistry(registry);
   }

   public void updateYawPitchRoll()
   {
      rootJoint.getYawPitchRoll(yaw, pitch, roll);
   }

   public void getTransformFromWorld(Transform3D transform3D)
   {
      rootJoint.getTransformFromWorld(transform3D);
   }

   public void setXYZ(double x, double y, double z, double xDot, double yDot, double zDot)
   {
      rootJoint.setXYZ(x, y, z, xDot, yDot, zDot);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll, double qd_wx, double qd_wy, double qd_wz)
   {
      rootJoint.setYawPitchRoll(yaw, pitch, roll, qd_wx, qd_wy, qd_wz);
   }

}
