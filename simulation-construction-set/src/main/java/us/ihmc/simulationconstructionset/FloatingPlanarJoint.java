package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.Plane;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.FloatingPlanarJointPhysics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariableList;

public class FloatingPlanarJoint extends Joint implements FloatingSCSJoint
{
   private static final long serialVersionUID = -1627814016079577790L;

   public YoDouble q_t1;
   public YoDouble q_t2;
   public YoDouble qd_t1;
   public YoDouble qd_t2;
   public YoDouble q_rot;
   public YoDouble qd_rot;
   public YoDouble qdd_t1, qdd_t2, qdd_rot;
   public Plane type = Plane.XZ;

   public YoDouble getQ_t1()
   {
      return q_t1;
   }

   public YoDouble getQ_t2()
   {
      return q_t2;
   }

   public YoDouble getQd_t1()
   {
      return qd_t1;
   }

   public YoDouble getQd_t2()
   {
      return qd_t2;
   }

   public YoDouble getQ_rot()
   {
      return q_rot;
   }

   public YoDouble getQd_rot()
   {
      return qd_rot;
   }

   public YoDouble getQdd_t1()
   {
      return qdd_t1;
   }

   public YoDouble getQdd_t2()
   {
      return qdd_t2;
   }

   public YoDouble getQdd_rot()
   {
      return qdd_rot;
   }

   YoVariableList floatingJointVars;

   public FloatingPlanarJoint(String jname, Robot rob)
   {
      this(jname, rob, Plane.XZ);
      physics = new FloatingPlanarJointPhysics(this);
   }

   public FloatingPlanarJoint(String jname, Vector3DReadOnly offset, Robot rob)
   {
      this(jname, offset, rob, Plane.XZ);
      physics = new FloatingPlanarJointPhysics(this);
   }

   public FloatingPlanarJoint(String jname, Robot rob, Plane type)
   {
      this(jname, new Vector3D(), rob, type);
   }

   public FloatingPlanarJoint(String jname, Vector3DReadOnly offset, Robot rob, Plane type)
   {
      super(jname, offset, rob, 3);
      physics = new FloatingPlanarJointPhysics(this);

      this.type = type;

      floatingJointVars = new YoVariableList(jname + " jointVars"); // rob.getVars();

      String t1_name, t2_name, rot_name;

      if (type == Plane.XY)
      {
         t1_name = "x";
         t2_name = "y";
         rot_name = "yaw";
      }
      else if (type == Plane.XZ)
      {
         t1_name = "x";
         t2_name = "z";
         rot_name = "pitch";
      }
      else
      {
         t1_name = "y";
         t2_name = "z";
         rot_name = "roll";
      }

      YoVariableRegistry registry = rob.getRobotsYoVariableRegistry();

      q_t1 = new YoDouble("q_" + t1_name, "PlanarFloatingJoint " + t1_name + " position", registry);
      q_t2 = new YoDouble("q_" + t2_name, "PlanarFloatingJoint " + t2_name + " position", registry);
      q_rot = new YoDouble("q_" + rot_name, "PlanarFloatingJoint " + rot_name + " angle", registry);

      qd_t1 = new YoDouble("qd_" + t1_name, "PlanarFloatingJoint " + t1_name + " linear velocity", registry);
      qd_t2 = new YoDouble("qd_" + t2_name, "PlanarFloatingJoint " + t2_name + " linear velocity", registry);
      qd_rot = new YoDouble("qd_" + rot_name, "PlanarFloatingJoint " + rot_name + " angular velocity", registry);

      qdd_t1 = new YoDouble("qdd_" + t1_name, "PlanarFloatingJoint " + t1_name + " linear acceleration", registry);
      qdd_t2 = new YoDouble("qdd_" + t2_name, "PlanarFloatingJoint " + t2_name + " linear acceleration", registry);
      qdd_rot = new YoDouble("qdd_" + rot_name, "PlanarFloatingJoint " + rot_name + " angular acceleration", registry);

      //    rob.getVars().addVariables(floatingJointVars);

      this.setFloatingTransform3D(this.jointTransform3D);

      physics.u_i = null;
   }

   public FloatingPlanarJoint(String jname, String varName, Robot rob, Plane type)
   {
      this(jname, varName, new Vector3D(), rob, type);
   }

   public FloatingPlanarJoint(String jname, String varName, Vector3D offset, Robot rob, Plane type)
   {
      super(jname, offset, rob, 6);
      physics = new FloatingPlanarJointPhysics(this);

      this.type = type;

      floatingJointVars = new YoVariableList(jname + " jointVars"); // rob.getVars();

      String t1_name, t2_name, rot_name;

      if (type == Plane.XY)
      {
         t1_name = varName + "_x";
         t2_name = varName + "_y";
         rot_name = varName + "_yaw";
      }
      else if (type == Plane.XZ)
      {
         t1_name = varName + "x";
         t2_name = varName + "z";
         rot_name = varName + "pitch";
      }
      else
      {
         t1_name = varName + "y";
         t2_name = varName + "z";
         rot_name = varName + "roll";
      }

      YoVariableRegistry registry = rob.getRobotsYoVariableRegistry();

      q_t1 = new YoDouble("q_" + t1_name, "PlanarFloatingJoint " + t1_name + " position", registry);
      q_t2 = new YoDouble("q_" + t2_name, "PlanarFloatingJoint " + t2_name + " position", registry);
      q_rot = new YoDouble("q_" + rot_name, "PlanarFloatingJoint " + rot_name + " angle", registry);

      qd_t1 = new YoDouble("qd_" + t1_name, "PlanarFloatingJoint " + t1_name + " linear velocity", registry);
      qd_t2 = new YoDouble("qd_" + t2_name, "PlanarFloatingJoint " + t2_name + " linear velocity", registry);
      qd_rot = new YoDouble("qd_" + rot_name, "PlanarFloatingJoint " + rot_name + " angular velocity", registry);

      qdd_t1 = new YoDouble("qdd_" + t1_name, "PlanarFloatingJoint " + t1_name + " linear acceleration", registry);
      qdd_t2 = new YoDouble("qdd_" + t2_name, "PlanarFloatingJoint " + t2_name + " linear acceleration", registry);
      qdd_rot = new YoDouble("qdd_" + rot_name, "PlanarFloatingJoint " + rot_name + " angular acceleration", registry);

      //    rob.getVars().addVariables(floatingJointVars);

      this.setFloatingTransform3D(this.jointTransform3D);

      physics.u_i = null;
   }

   public void setCartesianPosition(double t1, double t2)
   {
      q_t1.set(t1);
      q_t2.set(t2);
   }

   public void setCartesianPosition(double t1, double t2, double t1Dot, double t2Dot)
   {
      q_t1.set(t1);
      q_t2.set(t2);
      qd_t1.set(t1Dot);
      qd_t2.set(t2Dot);
   }

   public void setCartesianPosition(Tuple2DReadOnly position, Tuple2DReadOnly velocity)
   {
      q_t1.set(position.getX());
      q_t2.set(position.getY());
      qd_t1.set(velocity.getX());
      qd_t2.set(velocity.getY());
   }

   public void setCartesianVelocity(Tuple2DReadOnly velocity)
   {
      qd_t1.set(velocity.getX());
      qd_t2.set(velocity.getY());
   }

   public void setCartesianVelocity(double t1Dot, double t2Dot)
   {
      qd_t1.set(t1Dot);
      qd_t2.set(t2Dot);
   }

   public void setRotation(double theta)
   {
      this.q_rot.set(theta);
   }

   public void setRotation(double theta, double thetaDot)
   {
      this.q_rot.set(theta);
      this.qd_rot.set(thetaDot);
   }

   public void setRotationalVelocity(double thetaDot)
   {
      this.qd_rot.set(thetaDot);
   }

   protected YoVariableList getJointVars()
   {
      return this.floatingJointVars;
   }

   @Override
   protected void update()
   {
      this.setFloatingTransform3D(this.jointTransform3D);
   }

   private Vector3D position = new Vector3D();

   protected void setFloatingTransform3D(RigidBodyTransform t1)
   {
      if (type == Plane.YZ)
      {
         position.set(0.0, q_t1.getDoubleValue(), q_t2.getDoubleValue());
         t1.setRotationRollAndZeroTranslation(q_rot.getDoubleValue());
      }
      else if (type == Plane.XZ)
      {
         position.set(q_t1.getDoubleValue(), 0.0, q_t2.getDoubleValue());
         t1.setRotationPitchAndZeroTranslation(q_rot.getDoubleValue());
      }
      else
      {
         position.set(q_t1.getDoubleValue(), q_t2.getDoubleValue(), 0.0);
         t1.setRotationYawAndZeroTranslation(q_rot.getDoubleValue());
      }

      t1.setTranslation(position);
   }

   public Plane getType()
   {
      return type;
   }

   private final double[] yawPitchRoll = new double[3];

   @Override
   public void setRotationAndTranslation(RigidBodyTransform transform)
   {
      Quaternion rotation = new Quaternion();
      transform.getRotation(rotation);

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(rotation, yawPitchRoll);

      Vector3D translation = new Vector3D();
      transform.getTranslation(translation);

      switch (type)
      {
      case XY:
         setRotation(yawPitchRoll[2]);
         setCartesianPosition(translation.getX(), translation.getY());
         break;
      case XZ:
         setRotation(yawPitchRoll[1]);
         setCartesianPosition(translation.getX(), translation.getZ());
         break;
      default:
         setRotation(yawPitchRoll[0]);
         setCartesianPosition(translation.getY(), translation.getZ());
         break;
      }
   }

   @Override
   public void setVelocity(Tuple3DReadOnly velocity)
   {
      switch (type)
      {
      case XY:
         setCartesianVelocity(velocity.getX(), velocity.getY());
         break;
      case XZ:
         setCartesianVelocity(velocity.getX(), velocity.getZ());
         break;
      default:
         setCartesianVelocity(velocity.getY(), velocity.getZ());
         break;
      }
   }

   @Override
   public void setAngularVelocityInBody(Vector3DReadOnly angularVelocityInBody)
   {
      switch (type)
      {
      case XY:
         setRotationalVelocity(angularVelocityInBody.getZ());
         break;
      case XZ:
         setRotationalVelocity(angularVelocityInBody.getY());
         break;
      default:
         setRotationalVelocity(angularVelocityInBody.getX());
         break;
      }
   }

   @Override
   public void getVelocity(FrameVector3D linearVelocityToPack)
   {
      switch (type)
      {
      case XY:
         linearVelocityToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), qd_t1.getDoubleValue(), qd_t2.getDoubleValue(), 0.0);
         break;
      case XZ:
         linearVelocityToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), qd_t1.getDoubleValue(), 0.0, qd_t2.getDoubleValue());
         break;
      default:
         linearVelocityToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, qd_t1.getDoubleValue(), qd_t2.getDoubleValue());
         break;
      }
   }

   @Override
   public void getAngularVelocity(FrameVector3D angularVelocityToPack, ReferenceFrame bodyFrame)
   {
      switch (type)
      {
      case XY:
         angularVelocityToPack.setIncludingFrame(bodyFrame, 0.0, 0.0, qd_rot.getDoubleValue());
         break;
      case XZ:
         angularVelocityToPack.setIncludingFrame(bodyFrame, 0.0, qd_rot.getDoubleValue(), 0.0);
         break;
      default:
         angularVelocityToPack.setIncludingFrame(bodyFrame, qd_rot.getDoubleValue(), 0.0, 0.0);
         break;
      }
   }
}
