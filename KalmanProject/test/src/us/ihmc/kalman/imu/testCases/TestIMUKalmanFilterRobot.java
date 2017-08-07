package us.ihmc.kalman.imu.testCases;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

public class TestIMUKalmanFilterRobot extends Robot
{
   /**
    *
    */
   private static final long serialVersionUID = -2714371959408634768L;

   // Primary joints of the robot...
   public FloatingJoint rootJoint;

   public final YoDouble q_x, q_y, q_z;    // Position in world coordinates
   public final YoDouble qd_x, qd_y, qd_z;    // Velocity in world coordinates.
   public final YoDouble qdd_x, qdd_y, qdd_z;    // Acceleration in world coordinates.

   public final YoDouble q_qs, q_qx, q_qy, q_qz;    // Rotation as a quaternion.
   public final YoDouble qd_wx, qd_wy, qd_wz;    // Angular velocities in world coordinates
   public final YoDouble qdd_wx, qdd_wy, qdd_wz;    // Angular acceleration in world coordinates

   // Rotation of the body
   private final YoVariableRegistry registry = this.getRobotsYoVariableRegistry();
   public final YoDouble yaw = new YoDouble("yaw", registry);
   public final YoDouble pitch = new YoDouble("pitch", registry);
   public final YoDouble roll = new YoDouble("roll", registry);

   public final YoDouble ef_body_fx, ef_body_fy, ef_body_fz;    // External Forces in world coordinates

   public TestIMUKalmanFilterRobot()
   {
      super("TestIMURobot");

      rootJoint = new FloatingJoint("root", new Vector3D(0.0, 0.0, 0.0), this);

      Link link = new Link("ahrs");
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.2);
      linkGraphics.translate(0.0, 0.0, -0.05);
      linkGraphics.addCube(0.1, 0.1, 0.1);
      link.setLinkGraphics(linkGraphics);

      link.setMass(1.0);
      link.setMomentOfInertia(1.0, 1.0, 1.0);

      ExternalForcePoint ef_body = new ExternalForcePoint("ef_body", new Vector3D(), this.getRobotsYoVariableRegistry());
      rootJoint.addExternalForcePoint(ef_body);

      rootJoint.setLink(link);
      addRootJoint(rootJoint);

      q_x = (YoDouble)this.getVariable("q_x");
      q_y = (YoDouble)this.getVariable("q_y");
      q_z = (YoDouble)this.getVariable("q_z");

      qd_x = (YoDouble)this.getVariable("qd_x");
      qd_y = (YoDouble)this.getVariable("qd_y");
      qd_z = (YoDouble)this.getVariable("qd_z");

      qdd_x = (YoDouble)this.getVariable("qdd_x");
      qdd_y = (YoDouble)this.getVariable("qdd_y");
      qdd_z = (YoDouble)this.getVariable("qdd_z");

      q_qs = (YoDouble)this.getVariable("q_qs");
      q_qx = (YoDouble)this.getVariable("q_qx");
      q_qy = (YoDouble)this.getVariable("q_qy");
      q_qz = (YoDouble)this.getVariable("q_qz");

      qd_wx = (YoDouble)this.getVariable("qd_wx");
      qd_wy = (YoDouble)this.getVariable("qd_wy");
      qd_wz = (YoDouble)this.getVariable("qd_wz");

      qdd_wx = (YoDouble)this.getVariable("qdd_wx");
      qdd_wy = (YoDouble)this.getVariable("qdd_wy");
      qdd_wz = (YoDouble)this.getVariable("qdd_wz");

      ef_body_fx = (YoDouble)this.getVariable("ef_body_fx");
      ef_body_fy = (YoDouble)this.getVariable("ef_body_fy");
      ef_body_fz = (YoDouble)this.getVariable("ef_body_fz");

//      this.addYoVariableRegistry(registry);
   }

   public void updateYawPitchRoll()
   {
      rootJoint.getYawPitchRoll(yaw, pitch, roll);
   }

   public void getTransformFromWorld(RigidBodyTransform transform3D)
   {
      rootJoint.getTransformToWorld(transform3D);
   }

   public void setXYZ(double x, double y, double z, double xDot, double yDot, double zDot)
   {
      rootJoint.setPositionAndVelocity(x, y, z, xDot, yDot, zDot);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll, double qd_wx, double qd_wy, double qd_wz)
   {
      rootJoint.setYawPitchRoll(yaw, pitch, roll, qd_wx, qd_wy, qd_wz);
   }

}
