package us.ihmc.simulationconstructionset;

//import Jama.*;

import javax.vecmath.Tuple2d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.physics.engine.jerry.FloatingPlanarJointPhysics;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * Company:      Yobotics, Inc. <p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */
public class FloatingPlanarJoint extends Joint
{
   /**
    *
    */
   private static final long serialVersionUID = -1627814016079577790L;

   public static final int
      XY = 1, YZ = 2, XZ = 3;

   public DoubleYoVariable q_t1;
   public DoubleYoVariable q_t2;
   public DoubleYoVariable qd_t1;
   public DoubleYoVariable qd_t2;
   public DoubleYoVariable q_rot;
   public DoubleYoVariable qd_rot;
   public DoubleYoVariable qdd_t1, qdd_t2, qdd_rot;
   public int type = XZ;

   public DoubleYoVariable getQ_t1()
   {
	return q_t1;
   }
   public DoubleYoVariable getQ_t2()
   {
	return q_t2;
   }
   public DoubleYoVariable getQd_t1()
   {
	return qd_t1;
   }
   public DoubleYoVariable getQd_t2()
   {
	return qd_t2;
   }
   public DoubleYoVariable getQ_rot()
   {
	return q_rot;
   }
   public DoubleYoVariable getQd_rot()
   {
	return qd_rot;
   }
   public DoubleYoVariable getQdd_t1()
   {
	return qdd_t1;
   }
   public DoubleYoVariable getQdd_t2()
   {
	return qdd_t2;
   }
   public DoubleYoVariable getQdd_rot()
   {
	return qdd_rot;
   }

   YoVariableList floatingJointVars;

   public FloatingPlanarJoint(String jname, Robot rob)
   {
      this(jname, rob, XZ);
      physics = new FloatingPlanarJointPhysics(this);
   }

   public FloatingPlanarJoint(String jname, Robot rob, int type)
   {
      super(jname, new Vector3d(), rob, 3);
      physics = new FloatingPlanarJointPhysics(this);

      this.type = type;

      floatingJointVars = new YoVariableList(jname + " jointVars");    // rob.getVars();

      String t1_name, t2_name, rot_name;

      if (type == XY)
      {
         t1_name = "x";
         t2_name = "y";
         rot_name = "yaw";
      }
      else if (type == XZ)
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

      q_t1 = new DoubleYoVariable("q_" + t1_name, "PlanarFloatingJoint " + t1_name + " position", registry);
      q_t2 = new DoubleYoVariable("q_" + t2_name, "PlanarFloatingJoint " + t2_name + " position", registry);
      q_rot = new DoubleYoVariable("q_" + rot_name, "PlanarFloatingJoint " + rot_name + " angle", registry);

      qd_t1 = new DoubleYoVariable("qd_" + t1_name, "PlanarFloatingJoint " + t1_name + " linear velocity", registry);
      qd_t2 = new DoubleYoVariable("qd_" + t2_name, "PlanarFloatingJoint " + t2_name + " linear velocity", registry);
      qd_rot = new DoubleYoVariable("qd_" + rot_name, "PlanarFloatingJoint " + rot_name + " angular velocity", registry);

      qdd_t1 = new DoubleYoVariable("qdd_" + t1_name, "PlanarFloatingJoint " + t1_name + " linear acceleration", registry);
      qdd_t2 = new DoubleYoVariable("qdd_" + t2_name, "PlanarFloatingJoint " + t2_name + " linear acceleration", registry);
      qdd_rot = new DoubleYoVariable("qdd_" + rot_name, "PlanarFloatingJoint " + rot_name + " angular acceleration", registry);

//    rob.getVars().addVariables(floatingJointVars);

      this.setFloatingTransform3D(this.jointTransform3D);

      physics.u_i = null;
   }


   public FloatingPlanarJoint(String jname, String varName, Robot rob, int type)
   {
      super(jname, new Vector3d(), rob, 6);
      physics = new FloatingPlanarJointPhysics(this);

      this.type = type;

      floatingJointVars = new YoVariableList(jname + " jointVars");    // rob.getVars();

      String t1_name, t2_name, rot_name;

      if (type == XY)
      {
         t1_name = varName + "_x";
         t2_name = varName + "_y";
         rot_name = varName + "_yaw";
      }
      else if (type == XZ)
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

      q_t1 = new DoubleYoVariable("q_" + t1_name, "PlanarFloatingJoint " + t1_name + " position", registry);
      q_t2 = new DoubleYoVariable("q_" + t2_name, "PlanarFloatingJoint " + t2_name + " position", registry);
      q_rot = new DoubleYoVariable("q_" + rot_name, "PlanarFloatingJoint " + rot_name + " angle", registry);

      qd_t1 = new DoubleYoVariable("qd_" + t1_name, "PlanarFloatingJoint " + t1_name + " linear velocity", registry);
      qd_t2 = new DoubleYoVariable("qd_" + t2_name, "PlanarFloatingJoint " + t2_name + " linear velocity", registry);
      qd_rot = new DoubleYoVariable("qd_" + rot_name, "PlanarFloatingJoint " + rot_name + " angular velocity", registry);

      qdd_t1 = new DoubleYoVariable("qdd_" + t1_name, "PlanarFloatingJoint " + t1_name + " linear acceleration", registry);
      qdd_t2 = new DoubleYoVariable("qdd_" + t2_name, "PlanarFloatingJoint " + t2_name + " linear acceleration", registry);
      qdd_rot = new DoubleYoVariable("qdd_" + rot_name, "PlanarFloatingJoint " + rot_name + " angular acceleration", registry);

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

   public void setCartesianPosition(Tuple2d position, Tuple2d velocity)
   {
      q_t1.set(position.x);
      q_t2.set(position.y);
      qd_t1.set(velocity.x);
      qd_t2.set(velocity.y);
   }
   
   public void setCartesianVelocity(Tuple2d velocity)
   {
      qd_t1.set(velocity.getX());
      qd_t2.set(velocity.getY());
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

   protected void update()
   {
      this.setFloatingTransform3D(this.jointTransform3D);
   }


   private Vector3d position = new Vector3d();

   protected void setFloatingTransform3D(RigidBodyTransform t1)
   {
      if (type == YZ)
      {
         position.set(0.0, q_t1.getDoubleValue(), q_t2.getDoubleValue());
         t1.rotX(q_rot.getDoubleValue());
      }
      else if (type == XZ)
      {
         position.set(q_t1.getDoubleValue(), 0.0, q_t2.getDoubleValue());
         t1.rotY(q_rot.getDoubleValue());
      }
      else
      {
         position.set(q_t1.getDoubleValue(), q_t2.getDoubleValue(), 0.0);
         t1.rotZ(q_rot.getDoubleValue());
      }

      t1.setTranslation(position);
   }


   public int getType()
   {
      return type;
   }

}
