package us.ihmc.simulationconstructionset;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;

/**
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * Company:      Yobotics, Inc. <p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */


public class GimbalJoint extends PinJoint
{
   /**
    *
    */
   private static final long serialVersionUID = 6692640300004794312L;
   private PinJoint joint2, joint3;

   public GimbalJoint(String jname1, String jname2, String jname3, Vector3d offset, Robot rob, Axis firstAxis, Axis secondAxis, Axis thirdAxis)
   {
      super(jname1, offset, rob, firstAxis);

      joint2 = new PinJoint(jname2, new Vector3d(), rob, secondAxis);
      joint3 = new PinJoint(jname3, new Vector3d(), rob, thirdAxis);

      // super.addJoint(joint2); // This crashes.  Instead, add the joint manually:

      joint2.parentJoint = this;
      childrenJoints.add(joint2);

      // Set the child r_in value:

      joint2.physics.r_in.setX(0.0);
      joint2.physics.r_in.setY(0.0);
      joint2.physics.r_in.setZ(0.0);

      // joint2.addJoint(joint3); // This crashes.  Instead add the joint manually:

      joint3.parentJoint = joint2;
      joint2.childrenJoints.add(joint3);

      joint3.physics.r_in.setX(0.0);
      joint3.physics.r_in.setY(0.0);
      joint3.physics.r_in.setZ(0.0);
   }


   @Override
   public void addJoint(Joint nextJoint)
   {
      joint3.addJoint(nextJoint);
   }

   @Override
   public void setLink(Link l)
   {
      // Set this joints real link to a null link and set the second Joints link to the given link...
      Link nullLink = new Link("null");
      nullLink.setMass(0.0);
      nullLink.setMomentOfInertia(0.0, 0.0, 0.0);
      nullLink.setComOffset(0.0, 0.0, 0.0);

      super.setLink(nullLink);

      nullLink = new Link("null");
      nullLink.setMass(0.0);
      nullLink.setMomentOfInertia(0.0, 0.0, 0.0);
      nullLink.setComOffset(0.0, 0.0, 0.0);
      joint2.setLink(nullLink);

      joint3.setLink(l);
   }

   @Override
   public void addCameraMount(CameraMount mount)
   {
      joint3.addCameraMount(mount);
   }
   
   @Override
   public void addIMUMount(IMUMount mount)
   {
      joint3.addIMUMount(mount);
   }

   @Override
   public void addKinematicPoint(KinematicPoint point)
   {
      joint3.addKinematicPoint(point);
   }

   @Override
   public void addGroundContactPoint(GroundContactPoint point)
   {
      joint3.addGroundContactPoint(point);
   }

   @Override
   public void addExternalForcePoint(ExternalForcePoint point)
   {
      joint3.addExternalForcePoint(point);
   }

   public void setLimitStops(int axis, double q_min, double q_max, double k_limit, double b_limit)
   {
      if (axis == 1)
         super.setLimitStops(q_min, q_max, k_limit, b_limit);
      else if (axis == 2)
         joint2.setLimitStops(q_min, q_max, k_limit, b_limit);
      else if (axis == 3)
         joint3.setLimitStops(q_min, q_max, k_limit, b_limit);
   }

   public void setDamping(int axis, double b_damp)
   {
      if (axis == 1)
         super.setDamping(b_damp);
      else if (axis == 2)
         joint2.setDamping(b_damp);
      else if (axis == 3)
         joint3.setDamping(b_damp);
   }

   @Override
   public void setDamping(double b_damp)
   {
      super.setDamping(b_damp);
      joint2.setDamping(b_damp);
      joint3.setDamping(b_damp);
   }

   public void setInitialState(double q1_init, double qd1_init, double q2_init, double qd2_init, double q3_init, double qd3_init)
   {
      super.setInitialState(q1_init, qd1_init);
      joint2.setInitialState(q2_init, qd2_init);
      joint3.setInitialState(q3_init, qd3_init);
   }

   @Override
   public void getState(double[] state)
   {
      state[0] = q.getDoubleValue();
      state[1] = qd.getDoubleValue();
      state[2] = joint2.q.getDoubleValue();
      state[3] = joint2.qd.getDoubleValue();
      state[4] = joint3.q.getDoubleValue();
      state[5] = joint3.qd.getDoubleValue();
   }



}
