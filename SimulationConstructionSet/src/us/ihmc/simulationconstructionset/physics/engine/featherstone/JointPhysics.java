package us.ihmc.simulationconstructionset.physics.engine.featherstone;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.ExternalTorque;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.JointWrenchSensor;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SpatialVector;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;


public abstract class JointPhysics< J extends Joint>
{
   protected J owner;

   public Vector3D u_i;    // u_i is unit vector in direction of joint axis.  Set once upon creation by the specific joint.
   public Vector3D d_i = new Vector3D();    // d_i is vector from link i inboard joint to link i com (link.getComOffset());

   public Vector3D u_iXd_i = new Vector3D();    // u_iXd_i is u_i cross d_i.

   public Vector3D r_in = new Vector3D();    // r_in is the vector from the previous center of mass to this joint in the previous joint coordinates.

   public Vector3D w_i = new Vector3D();    // w_i is the rotational velocity of this link, in these coordinates.
   public Vector3D v_i = new Vector3D();    // v_i is the linear velocity of this link in these coordinates.

   public Vector3D r_i = new Vector3D();    // r_i is radius vector from previous center of mass to this center of mass in these coordinates.
   protected Vector3D r_h = new Vector3D();    // r_h is radius vector from this center of mass to the previous center of mass in the previous coordinates.
   protected Vector3D v_i_temp = new Vector3D();

   protected SpatialTransformationMatrix i_X_hat_h = new SpatialTransformationMatrix();
   protected SpatialTransformationMatrix h_X_hat_i = new SpatialTransformationMatrix();

   public SpatialVector Z_hat_i = new SpatialVector();
   public SpatialInertiaMatrix I_hat_i = new SpatialInertiaMatrix();
   public SpatialVector c_hat_i = new SpatialVector();
   public SpatialVector a_hat_i = new SpatialVector();

   public SpatialVector s_hat_i = new SpatialVector();    // Spatial Joint Axis
   public double Q_i;    // Joint Torque.

   public ArrayList<KinematicPoint> kinematicPoints;
   protected ArrayList<ExternalForcePoint> externalForcePoints;
   protected ArrayList<ExternalTorque> externalTorques;

   public LinkedHashMap<Integer, GroundContactPointGroup> groundContactPointGroups;
   public ArrayList<GroundContactPointGroup> groundContactPointGroupList;

   protected JointWrenchSensor jointWrenchSensor;
   private SpatialVector tempJointWrenchVector;
   private Vector3D tempVectorForWrenchTranslation, tempOffsetForWrenchTranslation;

   // private Transform3D R = new Transform3D();
   public RotationMatrix Ri_0 = new RotationMatrix();    // Rotation matrix from this space to world space
   public RotationMatrix R0_i = new RotationMatrix();    // Rotation matrix from world space to this space
   public RotationMatrix Ri_h = new RotationMatrix();    // Rotation matrix from this joint to previous joint
   public RotationMatrix Rh_i = new RotationMatrix();    // Rotation matrix from previous joint to this joint
   public RotationMatrix Rtemp = new RotationMatrix();    // @todo Remove this if it is unused.

   public JointPhysics(J owner)
   {
      this.owner = owner;
   }

   /**
    * This featherstone pass recurses down the tree updating the linear and angular velocities
    * of each link based on their current position and velocity and the position and velocity of
    * the previous joint in the chain.  In the process of carrying out these calculations the
    * rotation matricies between frame i and the world, the world and frame i, frame i and frame i-1,
    * and frame i-1 and frame i are updated.  The vector r_i which decsribes the translation
    * between frames i and i-1 in frame i's coordinates is also updated.  The final step of this
    * pass is to update the ground contact and kinematic points with the new position, velocity,
    * and rotation data.  For further details see Brian Mirtich's Thesis: "Impulse-based
    * Dynamic Simulation of Rigid Body Systems".
    *
    * @param w_h Angular velocity of joint i-1.
    * @param v_h Linear velocity of joint i-1.
    * @param Rh_0 Rotation matrix between frame i-1 and the world.
    */
   public void featherstonePassOne(Vector3D w_h, Vector3D v_h, RotationMatrix Rh_0)
   {
      // First set the transform (rotation matrix) for this joint.
      // (R <- rotation matrix from F_h to F_i

      // this.update(false);
      // this.jointTransform3D.get(Rh_i);

      r_in.set(owner.offset);
      if (owner.parentJoint != null)
      {
         r_in.sub(owner.parentJoint.link.getComOffset());
      }

      this.jointDependentSetAndGetRotation(Rh_i);

      // R.setTranslation(new Vector3d());
      Ri_h.set(Rh_i);
      Ri_h.transpose();

      Ri_0.set(Ri_h);
      Ri_0.multiply(Rh_0);

      jointDependentSet_d_i();

      this.u_iXd_i.cross(u_i, d_i);

      // Now compute the radius vector
      // (r <- radius vector from F_h to F_i (in F_i coordinates)
      r_i.set(r_in);

      Ri_h.transform(r_i);
      r_i.add(d_i);

      r_h.set(r_i);
      r_h.scale(-1.0);
      Rh_i.transform(r_h);


      // w_i <- R w_h
      w_i.set(w_h);
      Ri_h.transform(w_i);

      // v_i <- R v_h + w_i X r
      v_i.set(v_h);
      Ri_h.transform(v_i);

      v_i_temp.cross(w_i, r_i);
      v_i.add(v_i_temp);


      // Now do the joint dependent stuff...

      this.jointDependentFeatherstonePassOne();


      // Now update the points attached to the joint:
      R0_i.set(Ri_0);
      R0_i.transpose();

      // Do not iterate over the groundContactPointGroups hashmap here, has a very large overhead!
      if (groundContactPointGroupList != null)
      {
         for (int i = 0; i < groundContactPointGroupList.size(); i++)
         {
            ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroupList.get(i).getGroundContactPoints();
            for (int y = 0; y < groundContactPoints.size(); y++)
            {
               GroundContactPoint point = groundContactPoints.get(y);
               point.updatePointVelocity(R0_i, this.owner.link.comOffset, v_i, w_i);
            }
         }
      }

      if (kinematicPoints != null)
      {
         for (int i = 0; i < this.kinematicPoints.size(); i++)
         {
            KinematicPoint point = this.kinematicPoints.get(i);
            point.updatePointVelocity(R0_i, this.owner.link.comOffset, v_i, w_i);
         }
      }

      // Recurse over the children:

      for (int i = 0; i < owner.getChildrenJoints().size(); i++)
      {
         Joint child = owner.getChildrenJoints().get(i);
         child.physics.featherstonePassOne(w_i, v_i, Ri_0);
      }
   }

   // Vectors used in the application of external force and ground contact points
   private final Vector3D externalForceVector = new Vector3D();
   private final Vector3D externalForceR = new Vector3D();
   private final Vector3D tempExternalMomentVector = new Vector3D();
   private final Vector3D externalMomentVector = new Vector3D();


   private final Vector3D w_h = new Vector3D();    // Velocity of the previous joint in this joints coordinates

   /**
    * The second Featherstone pass recurses down the tree initializing the spatial isolated inerta (spatial matrix) and
    * spatial isolated zero-acceleration (z.a.) force (spatial vector) of each link.  During this process the coriolis vector
    * for each link is also calculated.  External force and ground contact points present at the current link are included
    * the spatial isolated inertia and isolated z.a. force.  For further details see Brian Mirtich's Thesis: "Impulse-based
    * Dynamic Simulation of Rigid Body Systems".
    *
    * @param w_previous The angular velocity of the previous joint represented in that joints coordinates.  This value is
    * used in the calculation of the coriolis vector.
    */
   public void featherstonePassTwo(Vector3D w_previous)
   {
      // First need to rotate w_previous into these coordinate system to get w_h:
      w_h.set(w_previous);
      if (Ri_h != null)
         Ri_h.transform(w_h);    // Note: FloatingJoint doesn't set Ri_h, therefore you have to do this check...

      // Z_hat_i.setInitArticulatedZeroAccel(this.link.mass, this.w_i, this.link.Ixx, this.link.Iyy, this.link.Izz, Ri_0, Robot.gX, Robot.gY, Robot.gZ);
      // I_hat_i.setInitArticulatedInertia(this.link.mass, this.link.Ixx, this.link.Iyy, this.link.Izz);

      Z_hat_i.setInitArticulatedZeroAccel(owner.link.getMass(), this.w_i, owner.link.Inertia, Ri_0, owner.rob.gravityX.getDoubleValue(), owner.rob.gravityY.getDoubleValue(),
            owner.rob.gravityZ.getDoubleValue());
      I_hat_i.setInitArticulatedInertia(owner.link.getMass(), owner.link.Inertia);

      // Put ground contact point effects in Z_hat:
      // Do not iterate over the groundContactPointGroups hashmap here, has a very large overhead!
      if (groundContactPointGroupList != null)
      {
         for (int i = 0; i < groundContactPointGroupList.size(); i++)
         {
            ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroupList.get(i).getGroundContactPoints();
            for (int y = 0; y < groundContactPoints.size(); y++)
            {
               GroundContactPoint point = groundContactPoints.get(y);
               applyForcesAndMomentsFromExternalForcePoints(point);
            }
         }
      }

      // Put external force effects in Z_hat:

      // +++JEP OPTIMIZE

      if (externalForcePoints != null)
      {
         for (int i = 0; i < this.externalForcePoints.size(); i++)
         {
            ExternalForcePoint point = this.externalForcePoints.get(i);

            // mark it as inactive so that it needs to be re-activated the next
            point.active = false;

            applyForcesAndMomentsFromExternalForcePoints(point);
         }
      }

      if (externalTorques != null)
      {
         for (int i = 0; i < this.externalTorques.size(); i++)
         {
            ExternalTorque p = this.externalTorques.get(i);

            // mark it as inactive so that it needs to be re-activated the next
            p.active = false;

            double tx = p.getTorqueX(), ty = p.getTorqueY(), tz = p.getTorqueZ();

            if ((tx != 0.0) || (ty != 0.0) || (tz != 0.0))    // +++JEP OPTIMIZE: Don't do the math if the forces are zero!
            {
               // not really a force, but recycle this variable
               externalForceVector.set(-tx, -ty, -tz);

               // Rotate it into Link Coordinates:

               Ri_0.transform(externalForceVector);

               // Add it to Z_hat:

//                   Z_hat_i.top.add(externalForceVector);  there is no force just torque
               Z_hat_i.bottom.add(externalForceVector);
            }
         }
      }

      this.jointDependentFeatherstonePassTwo(w_h);

      // System.out.println(name + ":");
      // System.out.println("Z_hat_i: " + Z_hat_i);
      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.featherstonePassTwo(w_i);
      }

      // Zero these temporary variables out so that rewindability tests which use reflexion don't pick them up as changed state variables.
      externalForceVector.set(0.0, 0.0, 0.0);
      externalForceR.set(0.0, 0.0, 0.0);
      tempExternalMomentVector.set(0.0, 0.0, 0.0);
      externalMomentVector.set(0.0, 0.0, 0.0);
   }


   private void applyForcesAndMomentsFromExternalForcePoints(ExternalForcePoint point)
   {
      if (!point.isForceAndMomentZero())    // +++JEP OPTIMIZE: Don't do the math if the forces are zero!
      {
         point.getForce(externalForceVector);
         externalForceVector.negate();

         // Rotate it into Link Coordinates:

         Ri_0.transform(externalForceVector);

//       externalForceR.sub(p.offset, this.link.comOffset);    // JEP+++ 12/19/01.  Something wrong here!

         point.getOffset(externalForceR);
         externalForceR.sub(owner.link.comOffset);

         // jointDependentComputeExternalForceR(p.offset, this.link.comOffset, externalForceR);
         externalMomentVector.cross(externalForceR, externalForceVector);
         point.getMoment(tempExternalMomentVector);
         Ri_0.transform(tempExternalMomentVector);

         externalMomentVector.sub(tempExternalMomentVector);

         // Add it to Z_hat:

         Z_hat_i.top.add(externalForceVector);
         Z_hat_i.bottom.add(externalMomentVector);
      }
   }

   private double sIs;    // sIs is s_hat_i_prime I_hat_A_i s_hat_i
   private double Qi_etc;    // Qi - s_hat_i_prime (Z_hat_i + I_hat_i * c_hat_i)

   // Temporary variables used in intermediary calculations
   protected SpatialInertiaMatrix
         SIM1 = new SpatialInertiaMatrix(), SIM2 = new SpatialInertiaMatrix();
   protected SpatialVector
         sV1 = new SpatialVector(), sV2 = new SpatialVector();

   /**
    * Pass three calculates the articulated versions of both the spatial inertia and spatial
    * zero-acceleration (z.a.) force traveling from the leaves to the root.  These calculations
    * are based on data generated by the preceding two passes.  This is a helper method for pass
    * three which is only called on root nodes.  For further details see Brian
    * Mirtich's Thesis: "Impulse-based Dynamic Simulation of Rigid Body Systems".
    */
   public void featherstonePassThree()
   {
      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.featherstonePassThree(I_hat_i, Z_hat_i);
      }

      sIs = I_hat_i.sIs(s_hat_i);

      sV1.set(c_hat_i);
      I_hat_i.multiply(sV1);

      sV1.add(Z_hat_i);
      double temp = s_hat_i.innerProduct(sV1);

      /*
       * System.out.println(name + ":\n");
       * System.out.println("s_hat_i: " + s_hat_i + "\n");
       * System.out.println("sV1: " + sV1 + "\n");
       * System.out.println("temp: " + temp + "\n");
       */

      Qi_etc = Q_i - temp;

      // System.out.println("Z_hat_i: " + Z_hat_i);
      // System.out.println("c_hat_i: " + c_hat_i);
      // System.out.println("u_i: " + u_i);
      // System.out.println("d_i: " + d_i);
      // System.out.println("s_hat_i: " + s_hat_i);
      // System.out.println("temp: " + temp);


   }

   /**
    * Pass three calculates the articulated versions of both the spatial inertia and spatial
    * zero-acceleration (z.a.) force traveling from the leaves to the root.  These calculations
    * are based on data generated by the preceding two passes.  This is the recursive method
    * which travels to the outermost link to begin calculations.  The pass for link I produces
    * the values for link i-1. For further details see Brian Mirtich's Thesis: "Impulse-based
    * Dynamic Simulation of Rigid Body Systems".
    */
   public void featherstonePassThree(SpatialInertiaMatrix I_hat_h, SpatialVector Z_hat_h)
   {
      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.featherstonePassThree(I_hat_i, Z_hat_i);
      }


      // Compute Spatial Transformation Matrix:

      i_X_hat_h.setFromOffsetAndRotation(r_i, Ri_h);
      h_X_hat_i.setFromOffsetAndRotation(r_h, Rh_i);

      sIs = I_hat_i.sIs(s_hat_i);

      // System.out.println(this.name);
      // System.out.println("sIs: " + sIs);

      // Calculate I_hat_A_i - IssI/sIs

      SIM1.IssI(I_hat_i, s_hat_i, sIs);
      SIM2.sub(I_hat_i, SIM1);

      // for(int i=1; i<1000;i++)
      // {
      h_X_hat_i.transformSpatialInertia(SIM2);

      // h_X_hat_i.multiplySpatialInertia(SIM2);
      // i_X_hat_h.postMultiplySpatialInertia(SIM2);
      // }

      I_hat_h.add(SIM2);

      // Now do Z_hat_h
      sV1.set(c_hat_i);
      I_hat_i.multiply(sV1);

      sV1.add(Z_hat_i);

      Qi_etc = Q_i - s_hat_i.innerProduct(sV1);

      sV2.set(s_hat_i);
      I_hat_i.multiply(sV2);
      sV2.scale(Qi_etc / sIs);

      sV1.add(sV2);

      h_X_hat_i.transform(sV1);

      Z_hat_h.add(sV1);

      // System.out.println(this.name + ":\n" + I_hat_h);

      // System.out.println("Z_hat_i: " + Z_hat_i);
      // System.out.println("c_hat_i: " + c_hat_i);
      // System.out.println("s_hat_i: " + s_hat_i);
   }

   protected SpatialVector X_hat_h_a_hat_h = new SpatialVector();
   private SpatialVector qdd_s_hat_i = new SpatialVector();

   // private SpatialInertiaMatrix

   /**
    * The fourth featherstone pass calculates the spatial acceleration vector and
    * scalar acceleration for each joint.  These values are based on the preceding
    * passes and the spatial acceleration of joint i-1.  Once calculated the scalar
    * value is stored for the Runge-Kutta sum.  For further details see Brian
    * Mirtich's Thesis: "Impulse-based Dynamic Simulation of Rigid Body Systems".
    *
    * @param a_hat_h The spatial acceleration vector for link i-1.
    * @param passNumber In order to perform RK4 (Runge-Kutta) the four featherstone
    * passes are called four times each simulation tick.  This pass uses the overall
    * pass to index these results as they are saved.
    * @throws UnreasonableAccelerationException
    */
   public void featherstonePassFour(SpatialVector a_hat_h, int passNumber) throws UnreasonableAccelerationException
   {
      X_hat_h_a_hat_h.set(a_hat_h);
      i_X_hat_h.transform(X_hat_h_a_hat_h);
      I_hat_i.multiply(X_hat_h_a_hat_h);


      // System.out.println(this.name + " a_hat_h: " + a_hat_h);
      // System.out.println(this.name + " X_hat_h_a_hat_h: " + X_hat_h_a_hat_h);
      // System.out.println(this.name + " I_hat_i: " + I_hat_i);

      double temp = s_hat_i.innerProduct(X_hat_h_a_hat_h);
      double qdd = (Qi_etc - temp) / sIs;

      // System.out.println(this.name + " Qi_etc: " + Qi_etc);
      // System.out.println(this.name + " sIs: " + sIs);
      // System.out.println(this.name + " qdd: " + qdd);


      this.jointDependentFeatherstonePassFour(qdd, passNumber);

      qdd_s_hat_i.set(s_hat_i);
      qdd_s_hat_i.scale(qdd);

      a_hat_i.set(a_hat_h);
      i_X_hat_h.transform(a_hat_i);
      a_hat_i.add(c_hat_i);
      a_hat_i.add(qdd_s_hat_i);

      // System.out.println(this.name + " a_hat_i: " + a_hat_i);

      // Check for unreasonable accelerations:
      if (!jointDependentVerifyReasonableAccelerations())
      {
         ArrayList<Joint> unreasonableAccelerationJoints = new ArrayList<Joint>();
         unreasonableAccelerationJoints.add(owner);

         throw new UnreasonableAccelerationException(unreasonableAccelerationJoints);
      }

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.featherstonePassFour(a_hat_i, passNumber);
      }

      if (this.jointWrenchSensor != null)
      {
         computeAndSetWrenchAtJoint();
      }
   }

   private final Vector3D tempDeltaPVector = new Vector3D();
   private final Vector3D tempOmegaCrossDeltaPVector = new Vector3D();
   private final Vector3D tempOmegaCrossOmegaCrossDeltaPVector = new Vector3D();

   public void getLinearVelocityInBody(Vector3D linearVelocityInBodyToPack, Vector3D pointInBody)
   {
      tempDeltaPVector.set(pointInBody);
      tempDeltaPVector.sub(owner.link.comOffset);

      linearVelocityInBodyToPack.cross(w_i, tempDeltaPVector);
      linearVelocityInBodyToPack.add(v_i);
   }

   public void getLinearVelocityInWorld(Vector3D linearVelocityInWorldToPack, Vector3D pointInBody)
   {
      getLinearVelocityInBody(linearVelocityInWorldToPack, pointInBody);
      owner.transformToNext.transform(linearVelocityInWorldToPack);
   }

   public void getAngularVelocityInBody(Vector3D angularVelocityInBodyToPack)
   {
      angularVelocityInBodyToPack.set(w_i);
   }

   public void getAngularVelocityInWorld(Vector3D angularVelocityInWorldToPack)
   {
      getAngularVelocityInBody(angularVelocityInWorldToPack);
      owner.transformToNext.transform(angularVelocityInWorldToPack);
   }

   public void getLinearAccelerationInBody(Vector3D linearAccelerationInBodyToPack, Vector3D pointInBody)
   {
      tempDeltaPVector.set(pointInBody);
      tempDeltaPVector.sub(owner.link.comOffset);

      tempOmegaCrossDeltaPVector.cross(w_i, tempDeltaPVector);
      tempOmegaCrossOmegaCrossDeltaPVector.cross(w_i, tempOmegaCrossDeltaPVector);

      linearAccelerationInBodyToPack.cross(a_hat_i.top, tempDeltaPVector);
      linearAccelerationInBodyToPack.add(a_hat_i.bottom);
      linearAccelerationInBodyToPack.add(tempOmegaCrossOmegaCrossDeltaPVector);
   }


   public void getLinearAccelerationInWorld(Vector3D linearAccelerationInWorldToPack, Vector3D pointInBody)
   {
      getLinearAccelerationInBody(linearAccelerationInWorldToPack, pointInBody);
      owner.transformToNext.transform(linearAccelerationInWorldToPack);
   }

   public void getAngularAccelerationsInBodyFrame(Vector3D angularAccelerationToPack)
   {
      a_hat_i.getTop(angularAccelerationToPack);
   }

   public void getAngularAccelerationsInWorld(Vector3D angularAccelerationInWorldToPack)
   {
      getAngularAccelerationsInBodyFrame(angularAccelerationInWorldToPack);
      owner.transformToNext.transform(angularAccelerationInWorldToPack);
   }

   /**
    * Records the K values for Runge-Kutta calculations on non-dynamic
    * joint trees.
    *
    * @param passNumber The current pass count, four coefficents are needed
    * for the Runge-Kutta method so passes 0 through 3 are valid.
    */
   public void recordK(int passNumber)
   {
      this.jointDependentRecordK(passNumber);

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recordK(passNumber);
      }
   }

   // Collision Stuff:

   /**
    * Recurses over the children and performes a Euler Integration on the
    * position and velocity parameters of each.  As joint types have different
    * parameters each must implement this method.  Each value is calculated
    * based on the following formula:<br />
    * y_(n+1) = y_n + h*f(t_n, y_n)  where h is the step size.
    *
    * @param stepSize Step size for the Euler Integration
    */
   public abstract void recursiveEulerIntegrate(double stepSize);

   /**
    * Saves the current state of each joint.  Different joint types have different relevant values, pin and slider joints
    * store only position and velocity.  These values are restored prior to each Euler integration in order to properly
    * calculate the Runge-Kutta slope.
    */
   public abstract void recursiveSaveTempState();

   /**
    * Restores the previous state for the relevant joint variables for use in the next Euler Integration.
    */
   public abstract void recursiveRestoreTempState();

   /**
    * This function recurses through the children of this joint calculating new values for the
    * relevant variables using the RK4 method.  This is based on the set of four values stored for
    * each variable during {@link Robot#doDynamics doDynamics}.
    * The RK4 method operates in the following form:
    *
    * y_(n+1) = y_n + 1/6 * h * (k_1 + 2k_2 + 2k_3 + k_4
    * t_(n+1) = t_n + h
    *
    * Where:
    *
    * h is the step size
    * k_1 = f(t_n, y_n)
    * k_2 = f(t_n + 0.5 * h, y_n + 0.5 * h * k_1)
    * k_3 = f(t_n + 0.5 * h, y_n + 0.5 * h * k_2)
    * k_4 = f(t_n + h, y_n + h * k_3)
    *
    * @param stepSize The step size h, for use in calculation.
    */
   public abstract void recursiveRungeKuttaSum(double stepSize);

   private Matrix3D Ki = new Matrix3D();
   private CollisionIntegrator collisionIntegrator = new CollisionIntegrator();

   // private Vector3d impulse = new Vector3d();
   private SpatialVector p_hat_k = new SpatialVector();

   private SpatialTransformationMatrix k_X_hat_coll = new SpatialTransformationMatrix();

   private Vector3D tempVector = new Vector3D();

   /**
    * This method calculates the multibody collision matrix Ki which relates the applied impulse
    * to the the change in contact point velocity.  This is solved for both bodies involved in the
    * collision and then combined to form the final matrix K.
    *
    * @param offsetFromCOM Location of the collision with respect to the joint center of mass
    * @param Rk_coll Rotation matrix from the body frame of the colliding link to the collision frame.
    * The collision frame is aligned with the z axis normal to the collision point.
    * @return Matrix3d Ki, which represents the half of the collision matrix.
    */
   public Matrix3D computeKiCollision(Vector3D offsetFromCOM, RotationMatrix Rk_coll)
   {
      tempVector.set(offsetFromCOM);
      tempVector.scale(-1.0);

      k_X_hat_coll.setFromOffsetAndRotation(tempVector, Rk_coll);

      computeMultibodyKi(k_X_hat_coll, Ki);    // Ki is in collision coordinates.
      return Ki;
   }

   /**
    * See Mirtich 3.3 p66
    *
    * This is the process of tracking u_coll, the contact point velocity.  The value of u_coll is
    * calculated over compression and restitution.  Once the change in u_coll is calculated the
    * desired reaction impluse is calculated and stored in p_coll as described by the following
    * equation:  delta_u_coll = Ki*p or Ki_inv*delta_u_coll = p
    *
    * @param Ki The combined collision matrix representing both points.
    * @param u_coll The relative contact point velocity between the two colliding bodies
    * @param epsilon The coefficent of restitution.
    * @param mu The coefficent of friction.
    * @param p_coll Spatial collision impulse
    */
   public void integrateCollision(Matrix3D Ki, Vector3D u_coll, double epsilon, double mu, Vector3D p_coll)
   {
      // Integrate the collision (Mirtich p. 146, step D.):

      // System.out.println("u_coll: ");System.out.println(u_coll);

      collisionIntegrator.setup(Ki, u_coll, epsilon, mu);
      collisionIntegrator.computeImpulse(p_coll);
   }

   /**
    * Converts the provided impluse from collision space into joint space and if the value
    * is not unreasonably large, applies the impulse to the system.
    *
    * @param p_coll Vector3d the impulse applied in reaction to the collision event.
    */
   public void applyImpulse(Vector3D p_coll)
   {
      /*
       * if(p_coll.z < 0.0)
       * {
       * // Something is wrong if the impulse has negative components in the z direction!!!
       * //System.out.println("Collision has negative z component!!!");
       * }
       */

      // impulse is in collision coordinates.  Need to rotate into joint com coordinates:

      // System.out.println("p_coll: ");System.out.println(p_coll);

      p_hat_k.top.set(p_coll);
      p_hat_k.bottom.set(0.0, 0.0, 0.0);
      k_X_hat_coll.transform(p_hat_k);    // p_k is now the impulse at the joint com.

      // System.out.println("r_k_coll: " + r_k_coll);
      // System.out.println("p_hat_k: " + p_hat_k);

      if (p_coll.length() < 1000000000.0)
         propagateImpulse(p_hat_k);    // Propagate the impulse.  Mirtich p. 146, step E.
      else
      {
         System.out.println("p_coll is enormous:  " + p_coll);

         // System.out.println("u_coll: " + u_coll);
         System.out.println("K: " + Ki);
      }
   }

   /**
    * Resolves the specified collision event by calculating the collision impulse and applying it to the link.
    * This process is completed over the course of several steps.  First the collision matrix Ki is computed
    * based on the three basis vectors of collision space. Once that calculation is completed a collision
    * integration is executed to compute the desired impulse, p_coll.  When that impulse is computed it is then
    * applied to the system and the changes in velocity are propagated back through the system.
    * For further information see Mirtich pg. 146
    *
    * @param offsetFromCOM Vector describing the distance between the link center of mass and the point of collision.
    * @param Rk_coll Rotation matrix from joint k to collision space.
    * @param u_coll Vector representing the velocity at the collision point.
    * @param epsilon The coefficent of restitution.
    * @param mu The coefficent of friction.
    * @param p_coll Vector to store the collision impulse once it has been calculated.
    */
   public void resolveCollision(Vector3D offsetFromCOM, RotationMatrix Rk_coll, Vector3D u_coll, double epsilon, double mu, Vector3D p_coll)
   {
      computeKiCollision(offsetFromCOM, Rk_coll);
      integrateCollision(Ki, u_coll, epsilon, mu, p_coll);
      applyImpulse(p_coll);
   }

   /**
    * Micro collisions are used to simulate static contact.  They prevent simulation failure when the separation distance
    * and time between collisions approaches zero (static contact).  This micro collisions only take place when the initial
    * velocities of the two bodies are quite small.  Micro collisions work by artifically increasing the coefficent of restitution
    * (epsilon) based on the penetration of the bodies in the collision envelope.  Once this new penetration coefficent
    * is calculated the collision is resolved normally.  A special case is the instance where the two bodies are resting statically
    * due to a high coefficent of friction.  Normal micro collisions would cause a slow creep down the slope which is not
    * an accurate model.  In this senario the impulse is simply the force required to reverse the velocity at the point of collision,
    * effectively countering the artifical sliding effect.  For further information see Mirtich 3.5 pg 88
    *
    * @param penetrationSquared Square of the distance between the two components.
    * @param offsetFromCOM Vector3d describing the offset between this link's center of mass and the point of collision
    * @param Rk_coll Rotation matrix between this space, link k, and collision space
    * @param u_coll Vector representing the velocity at the point of collision
    * @param epsilon The coefficent of restitution.
    * @param mu The coefficent of friction.
    * @param p_coll Vector3d in which the impulse will be stored.
    */
   public void resolveMicroCollision(double penetrationSquared, Vector3D offsetFromCOM, RotationMatrix Rk_coll, Vector3D u_coll, double epsilon, double mu,
                                     Vector3D p_coll)
   {
      //TODO: Commented out microcollisions for time being and replaced with penetration-based
      // epsilon increase. Need to make test cases and figure out exactly how we are going to 
      // do all of this...
      boolean justUseSpringyEpsilonForMicroCollisions = true;
      if (justUseSpringyEpsilonForMicroCollisions)
      {
         resolveMicroCollisionWithSpringyEpsilon(penetrationSquared, offsetFromCOM, Rk_coll, u_coll, epsilon, mu, p_coll);
         return;
      }

      computeKiCollision(offsetFromCOM, Rk_coll);
      collisionIntegrator.setup(Ki, u_coll, epsilon, mu);
      collisionIntegrator.computeMicroImpulse(p_coll);    // impulse is in collision coordinates.  Need to rotate into joint com coordinates:

      if (Math.abs(Math.sqrt(p_coll.getX() * p_coll.getX() + p_coll.getY() * p_coll.getY()) / p_coll.getZ()) > mu)    // If slipping, just do it the normal way...
      {
         // Slipping MicroCollision:
//         System.out.println("Slipping Micro Collision:  " + p_coll);
         resolveCollision(offsetFromCOM, Rk_coll, u_coll, epsilon, mu, p_coll);
      }
      else
      {
//         System.out.println("Non-Slipping Micro Collision:  " + p_coll);

         p_hat_k.top.set(p_coll);
         p_hat_k.bottom.set(0.0, 0.0, 0.0);
         k_X_hat_coll.transform(p_hat_k);    // p_k is now the impulse at the joint com.

         propagateImpulse(p_hat_k);    // Propagate the impulse.  Mirtich p. 146, step E.
      }


   }
   
   private void resolveMicroCollisionWithSpringyEpsilon(double penetrationSquared, Vector3D offsetFromCOM, RotationMatrix Rk_coll, Vector3D u_coll, double epsilon, double mu,
                                                        Vector3D p_coll)
   {
   // +++JEP: Adjust epsilon based on penetration...
      epsilon = epsilon + 1000000.0 * penetrationSquared;
      if (epsilon > 20.0)
         epsilon = 20.0;

//      System.out.println("epsilon2 = " + epsilon);
//      System.out.println("penetrationSquared2 = " + penetrationSquared);
      // if (epsilon > 1.2) System.out.print(epsilon + " ");

      resolveCollision(offsetFromCOM, Rk_coll, u_coll, epsilon, mu, p_coll);
   }

   private SpatialVector delta_v_hat_null = new SpatialVector();

   /**
    * Given an impulse at this links center of mass this function
    * computes the change in velocity of all links.
    *
    * @param p_hat_k SpatialVector describing the force and torque impulse to be applied at this links center of mass.
    */
   private void propagateImpulse(SpatialVector p_hat_k)
   {
      // Mirtich p. 145.  Given an impulse at the center of mass of this link,
      // compute the resulting change in velocity of _all_ links.

      Y_hat_parent.set(p_hat_k);
      Y_hat_parent.scale(-1.0);

      // System.out.println("In propagateImpulse");
      // System.out.println("Y_hat_parent: " + Y_hat_parent);

      // Recurse up the tree to the root to update the Y_hat of each joint:
      this.recursivePropagateImpulse(Y_hat_parent, delta_v_hat_null, null);

   }

   /**
    * Beginning with link k, the link at which the collision occured, calculate the new Y_hat for each parent, link h,
    * traveling up the tree to the root.  Once there, interate back down the tree calculating the change in velocity
    * for each node.  This downward traversal of the tree skips all child nodes in the path from k to the root as these
    * nodes are already updating their subtrees.
    *
    * @param new_Y_hat SpatialVector representing the force Z_hat_i must cancel as a result of the collision.
    * @param delta_v_hat_return SpatialVector representing the change in velocity of this joint due to the collison.
    * @param skipMeBackUp This joint is the child on the path to link k, skip it when calculating velocity changes
    * for joints not on the path.
    */
   protected void recursivePropagateImpulse(SpatialVector new_Y_hat, SpatialVector delta_v_hat_return, Joint skipMeBackUp)
   {
      Y_hat_i.set(new_Y_hat);

      // Go to the root updating the z.a. articulated impulses
      if (owner.parentJoint != null)
      {
         // Compute the parents Y_hat, as in Mirtich p. 141:

         Y_hat_parent.set(Y_hat_i);

         sIs = I_hat_i.sIs(s_hat_i);

         // SIM1.IssI(I_hat_i,s_hat_i,sIs);  // +++JEP This is wrong!!! This is how I had it, but then fixed it to be the next line!!!
         SIM1.Iss_sIs(I_hat_i, s_hat_i, sIs);

         // System.out.println("I_hat_i ss/ sIs for  " + this.name + " is " + SIM1);
         SIM1.oneMinus();

         SIM1.multiply(Y_hat_parent);
         h_X_hat_i.transform(Y_hat_parent);

         owner.parentJoint.physics.recursivePropagateImpulse(Y_hat_parent, delta_v_hat_h, owner);
      }
      else
      {
         delta_v_hat_h.top.set(0.0, 0.0, 0.0);
         delta_v_hat_h.bottom.set(0.0, 0.0, 0.0);
      }

      // Coming back up...

      // delta_v_hat_h now has the change of velocity of the parent joint so lets compute our change in velocity.
      // Note:  If we are a FloatingJoint, we need to use the equation on Mirtich p. 144 since there is no world to
      // take up some of the impulse.  We have to take it all up ourselves...
      // Therefore, we'll call a function here, and override it in the FloatingJoint and PlanarFloatingJoint...

      propagateImpulseSetDeltaVOnPath(delta_v_hat_h, delta_v_hat_return);

      // Iterate over the children, except for the path back up which gets iterated already...

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         if (child != skipMeBackUp)
            child.physics.recursivePropagateImpulseSetDeltaVOffPath(delta_v_hat_return);

         // delta_v_hat_return is my delta_v_hat.  Give it to my other children directly.
      }
   }

   private final SpatialVector p_hat_coll = new SpatialVector(), delta_v_hat_k = new SpatialVector();
   private final SpatialTransformationMatrix coll_X_hat_k = new SpatialTransformationMatrix();

   /**
    * Computes Ki for this joint using the three basis vectors describing collision space as test impulses.  The
    * calculated collision matrix is stored in Ki.  For further information see Mirtich p. 144
    *
    * @param k_X_hat_coll Spatial transformation matrix between this joint and collision space.
    * @param kiToPack Matrix3d in which the collision matrix will be stored.
    */
   private void computeMultibodyKi(SpatialTransformationMatrix k_X_hat_coll, Matrix3D kiToPack)
   {
      // Mirtich p. 144.  Compute the multi-body Ki for this Joint, given the SpatialTransformationMatrix from the contact frame to the joint COM frame.
      // Articulated inertias are already computed from the dynamics.  k_X_hat_coll is given.

      coll_X_hat_k.set(k_X_hat_coll);
      coll_X_hat_k.invert();

      // System.out.println("coll_X_hat_k:  " + coll_X_hat_k);
      // System.out.println("k_X_hat_coll:  " + k_X_hat_coll);

      // x axis:
      p_hat_coll.top.setX(1.0);
      p_hat_coll.top.setY(0.0);
      p_hat_coll.top.setZ(0.0);
      p_hat_coll.bottom.setX(0.0);
      p_hat_coll.bottom.setY(0.0);
      p_hat_coll.bottom.setZ(0.0);
      k_X_hat_coll.transform(p_hat_coll);

      this.impulseResponse(p_hat_coll, delta_v_hat_k);
      coll_X_hat_k.transform(delta_v_hat_k);
      kiToPack.setM00(delta_v_hat_k.bottom.getX());
      kiToPack.setM10(delta_v_hat_k.bottom.getY());
      kiToPack.setM20(delta_v_hat_k.bottom.getZ());

      // y axis:
      p_hat_coll.top.setX(0.0);
      p_hat_coll.top.setY(1.0);
      p_hat_coll.top.setZ(0.0);
      p_hat_coll.bottom.setX(0.0);
      p_hat_coll.bottom.setY(0.0);
      p_hat_coll.bottom.setZ(0.0);
      k_X_hat_coll.transform(p_hat_coll);

      this.impulseResponse(p_hat_coll, delta_v_hat_k);
      coll_X_hat_k.transform(delta_v_hat_k);
      kiToPack.setM01(delta_v_hat_k.bottom.getX());
      kiToPack.setM11(delta_v_hat_k.bottom.getY());
      kiToPack.setM21(delta_v_hat_k.bottom.getZ());

      // z axis:
      p_hat_coll.top.setX(0.0);
      p_hat_coll.top.setY(0.0);
      p_hat_coll.top.setZ(1.0);
      p_hat_coll.bottom.setX(0.0);
      p_hat_coll.bottom.setY(0.0);
      p_hat_coll.bottom.setZ(0.0);
      k_X_hat_coll.transform(p_hat_coll);

      this.impulseResponse(p_hat_coll, delta_v_hat_k);
      coll_X_hat_k.transform(delta_v_hat_k);
      kiToPack.setM02(delta_v_hat_k.bottom.getX());
      kiToPack.setM12(delta_v_hat_k.bottom.getY());
      kiToPack.setM22(delta_v_hat_k.bottom.getZ());


      // +++JEP Run a bunch of test cases here:

      /*
       * p_hat_coll.top.x = 1.0; p_hat_coll.top.y = 0.0; p_hat_coll.top.z = 0.0; p_hat_coll.bottom.x = 0.0; p_hat_coll.bottom.y = 0.0; p_hat_coll.bottom.z = 0.0;
       * this.impulseResponse(p_hat_coll, delta_v_hat_k);
       * System.out.println("delta_v_hat_k from (1 0 0 , 0 0 0) impulse:  " + delta_v_hat_k);
       */

   }

   protected final  SpatialVector Y_hat_i = new SpatialVector();
   private final SpatialVector Y_hat_parent = new SpatialVector();

   // SpatialVector delta_v_hat_i = new SpatialVector();

   /**
    * Calculates the resulting change in velocity for this link given an impulse
    * at its center of mass.  This is used in calculating the collision matrix
    * Ki based on three test impulses.
    *
    * @param p_hat_coll SpatialVector representing the impulse to this link's center of mass
    * @param delta_v_hat SpatialVector in which this links change in velocity is stored.
    */
   private void impulseResponse(SpatialVector p_hat_coll, SpatialVector delta_v_hat)
   {
      // Mirtich p. 141.  Given an impulse at the center of mass of this link,
      // compute the resulting change in velocity of this link.

      Y_hat_parent.set(p_hat_coll);
      Y_hat_parent.scale(-1.0);

      // System.out.println("In impulseResponse.  ");
      // System.out.println("Y_hat_parent: " + Y_hat_parent);


      // Recurse up the tree to the root to update the Y_hat of each joint:
      this.recursiveImpulseResponseToRootAndBack(Y_hat_parent, delta_v_hat);

   }


   private SpatialVector delta_v_hat_h = new SpatialVector();

   /**
    * Travels up the tree from this link updating the articulated impulses of each joint in the chain.
    * Once this is complete, the function calculates the change in velocity at each joint.
    *
    * @param new_Y_hat SpatialVector containing the new value for Y_hat_i for this link.
    * @param delta_v_hat_return SpatialVector containing the change in velocity for this link.
    */
   protected void recursiveImpulseResponseToRootAndBack(SpatialVector new_Y_hat, SpatialVector delta_v_hat_return)
   {
      // System.out.println("In Recursive Impulse Response to Root and Back for Joint " + this.name);
      // System.out.println("Setting Y_hat_i for " + this.name + " to " + new_Y_hat);
      Y_hat_i.set(new_Y_hat);

      // Go to the root updating the z.a. articulated impulses
      if (owner.parentJoint != null)
      {
         // Compute the parents Y_hat, as in Mirtich p. 141:

         Y_hat_parent.set(Y_hat_i);

         sIs = I_hat_i.sIs(s_hat_i);

         // System.out.println("sIs for " + this.name + " is " + sIs);
         // System.out.println("I_hat_i for " + this.name + " is " + I_hat_i);
         // System.out.println("s_hat_i for " + this.name + " is " + s_hat_i);

         // SIM1.IssI(I_hat_i,s_hat_i,sIs);  // +++JEP This is wrong!!! This is how I had it, but then fixed it to be the next line!!!
         SIM1.Iss_sIs(I_hat_i, s_hat_i, sIs);

         // System.out.println("I_hat_i ss/ sIs for  " + this.name + " is " + SIM1);

         SIM1.oneMinus();

         SIM1.multiply(Y_hat_parent);

         // System.out.println("h_X_hat_i for " + this.name + " is " + h_X_hat_i);
         h_X_hat_i.transform(Y_hat_parent);

         owner.parentJoint.physics.recursiveImpulseResponseToRootAndBack(Y_hat_parent, delta_v_hat_h);
      }
      else
      {
         delta_v_hat_h.top.set(0.0, 0.0, 0.0);
         delta_v_hat_h.bottom.set(0.0, 0.0, 0.0);
      }

      // delta_v_hat_h now has the change of velocity of the parent joint so lets compute our change in velocity.
      // Note:  If we are a FloatingJoint, we need to use the equation on Mirtich p. 144 since there is no world to
      // take up some of the impulse.  We have to take it all up ourselves...
      // Therefore, we'll call a function here, and override it in the FloatingJoint and PlanarFloatingJoint...

      impulseResponseComputeDeltaV(delta_v_hat_h, delta_v_hat_return);


   }

   private final SpatialVector delta_v_temp1 = new SpatialVector(), delta_v_temp2 = new SpatialVector();

   /**
    * Compute the change in velocity for this joint as described in Mirtich p. 141.
    * This function applies only to joints on the path between and including link k, the
    * joint involved in collision, and the root joint.  This function is identical to
    * PropagateImpulseSetDeltaVOnPath in every way
    * except it does not store the calculated delta_q_dot_i.  This function is used
    * in the calculation of Ki which requires the propagation of test impulses which should
    * not be recorded.
    *
    * @param delta_v_parent SpatialVector representing the change in velocity of the parent joint.
    * @param delta_v_me SpatialVector to which the change in velocity of this joint is stored.
    */
   protected void impulseResponseComputeDeltaV(SpatialVector delta_v_parent, SpatialVector delta_v_me)
   {
      // This is overridden with a floating joint.

      delta_v_temp1.set(delta_v_parent);
      i_X_hat_h.transform(delta_v_temp1);

      delta_v_temp2.set(delta_v_temp1);
      I_hat_i.multiply(delta_v_temp2);
      delta_v_temp2.add(Y_hat_i);

      double delta_q_dot_i = -1.0 * s_hat_i.innerProduct(delta_v_temp2) / sIs;

      delta_v_temp2.set(s_hat_i);
      delta_v_temp2.scale(delta_q_dot_i);

      delta_v_me.add(delta_v_temp1, delta_v_temp2);
   }

   /**
    * Compute and store the change in velocity for this joint as described in Mirtich p. 141.
    * This function applies only to joints on the path between and including link k, the
    * joint involved in collision, and the root joint.
    *
    * @param delta_v_parent SpatialVector representing the change in velocity of the parent joint.
    * @param delta_v_me SpatialVector to which the change in velocity of this joint is stored.
    */
   protected void propagateImpulseSetDeltaVOnPath(SpatialVector delta_v_parent, SpatialVector delta_v_me)
   {
      // This is overridden with a floating joint.

      delta_v_temp1.set(delta_v_parent);
      i_X_hat_h.transform(delta_v_temp1);

      delta_v_temp2.set(delta_v_temp1);
      I_hat_i.multiply(delta_v_temp2);
      delta_v_temp2.add(Y_hat_i);

      double delta_q_dot_i = -1.0 * s_hat_i.innerProduct(delta_v_temp2) / sIs;
      jointDependentChangeVelocity(delta_q_dot_i);

      delta_v_temp2.set(s_hat_i);
      delta_v_temp2.scale(delta_q_dot_i);

      delta_v_me.add(delta_v_temp1, delta_v_temp2);

   }

   /**
    * Recurses down the tree calculating and storing the change in velocity
    * of each joint as a result of a collision at link k. This function is
    * for use on joints not on the path from link k to root.  For further
    * information see Mirtich p. 141
    *
    * @param delta_v_parent SpatialVector representing the change in velocity of this joints parent.
    */
   private void recursivePropagateImpulseSetDeltaVOffPath(SpatialVector delta_v_parent)
   {
      // This is overridden with a floating joint.

      delta_v_temp1.set(delta_v_parent);
      i_X_hat_h.transform(delta_v_temp1);

      delta_v_temp2.set(delta_v_temp1);
      I_hat_i.multiply(delta_v_temp2);

      // delta_v_temp2.add(Y_hat_i);  // Off path Y_hat_i = 0;

      double delta_q_dot_i = -1.0 * s_hat_i.innerProduct(delta_v_temp2) / sIs;
      jointDependentChangeVelocity(delta_q_dot_i);

      delta_v_temp2.set(s_hat_i);
      delta_v_temp2.scale(delta_q_dot_i);

      delta_v_temp1.add(delta_v_temp2);

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursivePropagateImpulseSetDeltaVOffPath(delta_v_temp1);
      }
   }

   private final Point3D tempCOMPoint = new Point3D();

   /**
    * Calculates center of mass position as well as the total mass of all joints beginning with
    * this joint as root.
    *
    * @param comPoint Point3D representing the location of this subtree's center of mass in world coordinates.
    * @return Double indicating the total mass of this joint and its children.
    */
   public double recursiveComputeCenterOfMass(Point3D comPoint)
   {
      double totalMass = 0.0;
      comPoint.set(0.0, 0.0, 0.0);
      tempCOMPoint.set(0.0, 0.0, 0.0);

      // Add the children
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);

         double mass = child.physics.recursiveComputeCenterOfMass(tempCOMPoint);

         totalMass += mass;
         tempCOMPoint.scale(mass);
         comPoint.add(tempCOMPoint);
      }

      // Get this joints com:
      tempCOMPoint.set(owner.link.comOffset);

      // Transform to World Coords
      owner.transformToNext.transform(tempCOMPoint);

      // System.out.println("Joint: " + this.name + " (" + tempCOMPoint.x + ", " + tempCOMPoint.y + ", " + tempCOMPoint.z + ")  " + link.mass);
      // Scale and add to the sum:
      totalMass += owner.link.getMass();
      tempCOMPoint.scale(owner.link.getMass());
      comPoint.add(tempCOMPoint);

      // Scale back:
      if (totalMass > 0.0)
      {
         comPoint.scale(1.0 / totalMass);
      }
      else
         comPoint.set(0.0, 0.0, 0.0);

      // System.out.println("Joint Total: " + this.name + " (" + comPoint.x + ", " + comPoint.y + ", " + comPoint.z + ")  " + totalMass);

      return totalMass;
   }

   private Vector3D tempLinearMomentum = new Vector3D();

   /**
    * Computes the total linear momentum and mass of this subtree.
    *
    * @param linearMomentum Vector3d representing the total linear momentum of this subtree in world coordinates.
    * @return The total mass of this subtree.
    */
   public double recursiveComputeLinearMomentum(Vector3D linearMomentum)
   {
      double totalMass = 0.0;
      linearMomentum.set(0.0, 0.0, 0.0);
      tempLinearMomentum.set(0.0, 0.0, 0.0);

      // Add the children
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);

         double mass = child.physics.recursiveComputeLinearMomentum(tempLinearMomentum);
         totalMass += mass;

         linearMomentum.add(tempLinearMomentum);
      }

      // Get this joints linear momentum in its coordinates:
      tempLinearMomentum.set(this.v_i);
      tempLinearMomentum.scale(owner.link.getMass());

      // Transform to World Coords
      owner.transformToNext.transform(tempLinearMomentum);

      // System.out.println("Joint: " + this.name + " (" + tempCOMVel.x + ", " + tempCOMVel.y + ", " + tempCOMVel.z + ")  " + link.mass);
      // Scale and add to the sum:
      totalMass += owner.link.getMass();
      linearMomentum.add(tempLinearMomentum);

      // System.out.println("Joint Total: " + this.name + " (" + comVel.x + ", " + comVel.y + ", " + comVel.z + ")  " + totalMass);

      return totalMass;
   }


   private Vector3D tempAngularMomentum = new Vector3D();
   private Vector3D tempCOMVector = new Vector3D();

   /**
    * Calculates the total angular momentum of this subtree in world coordinates.
    * @param angularMomentum Vector3d in which the total angular momentum will be stored.
    */
   public void recursiveComputeAngularMomentum(Vector3D angularMomentum)
   {
      angularMomentum.set(0.0, 0.0, 0.0);
      tempAngularMomentum.set(0.0, 0.0, 0.0);

      // Add the children
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);

         child.physics.recursiveComputeAngularMomentum(tempAngularMomentum);
         angularMomentum.add(tempAngularMomentum);

         // System.out.println(child.name + " angular momentum about 0: " + tempAngularMomentum);
      }

      // Get this joints com:
      tempCOMPoint.set(owner.link.comOffset);

      // Transform to World Coords
      owner.transformToNext.transform(tempCOMPoint);
      tempCOMVector.set(tempCOMPoint);

      // Get this joint's linear momentum in its coordinates:
      tempLinearMomentum.set(this.v_i);
      tempLinearMomentum.scale(owner.link.getMass());

      // Transform to World Coords
      owner.transformToNext.transform(tempLinearMomentum);

      // Cross Product:
      tempAngularMomentum.cross(tempCOMVector, tempLinearMomentum);
      angularMomentum.add(tempAngularMomentum);

      // System.out.println(this.name + "  p = " + tempCOMVector);
      // System.out.println(this.name + "  mv = " + tempLinearMomentum);
      // System.out.println(this.name + "  p X mv = " + tempAngularMomentum);

      // Angular momentum of this joint about its center of mass in its coordinate system:
      tempAngularMomentum.set(this.w_i);
      owner.link.Inertia.transform(tempAngularMomentum);

      // Transform to World Coords
      owner.transformToNext.transform(tempAngularMomentum);

      // Add them together:
      angularMomentum.add(tempAngularMomentum);

   }

   private Vector3D tempRotationalEnergyVector = new Vector3D();

   /**
    * Calculates the total rotational kinetic energy of this subtree.
    *
    * @return The total rotational kinetic energy.
    */
   public double recursiveComputeRotationalKineticEnergy()    // 1/2 w^T I w
   {
      tempRotationalEnergyVector.set(this.w_i);
      owner.link.Inertia.transform(tempRotationalEnergyVector);

      double rotationalKineticEnergy = 0.5 * w_i.dot(tempRotationalEnergyVector);

      // Add the children

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         rotationalKineticEnergy = rotationalKineticEnergy + child.physics.recursiveComputeRotationalKineticEnergy();
      }

      return rotationalKineticEnergy;
   }

   /**
    * Calculates the total translational kinetic energy of this subtree.
    *
    * @return The translational kinetic energy of this subtree.
    */
   public double recursiveComputeTranslationalKineticEnergy()    // 1/2 m v^T v
   {
      double translationalKineticEnergy = 0.5 * v_i.dot(v_i) * owner.link.getMass();

      // Add the children

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         translationalKineticEnergy = translationalKineticEnergy + child.physics.recursiveComputeTranslationalKineticEnergy();
      }

      return translationalKineticEnergy;
   }

   private final Point3D tempPE_COMPoint = new Point3D();

   private boolean doFreezeFrame = false;

   /**
    * Calculates the total potential energy of this subtree due to gravity.
    *
    * @return The total gravitational potential energy of this subtree.
    */
   public double recursiveComputeGravitationalPotentialEnergy()    // m g h
   {
      tempPE_COMPoint.set(owner.link.comOffset);    // Get this joints com

      // Transform to World Coords
      owner.transformToNext.transform(tempPE_COMPoint);

      double gravitationalPotentialEnergy = owner.link.getMass()
            * (-owner.rob.gravityX.getDoubleValue() * tempPE_COMPoint.getX() - owner.rob.gravityY.getDoubleValue() * tempPE_COMPoint.getY()
            - owner.rob.gravityZ.getDoubleValue() * tempPE_COMPoint.getZ());

      // Add the children

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         gravitationalPotentialEnergy = gravitationalPotentialEnergy + child.physics.recursiveComputeGravitationalPotentialEnergy();
      }

      return gravitationalPotentialEnergy;
   }

   /**
    * Recurse through the tree deciding if any ground contact points are in contact.  This function
    * builds the list of points in contact to be used during dynamics.
    */
   public void recursiveDecideGroundContactPointsInContact()
   {
      if (groundContactPointGroups != null)
      {
         for (int i = 0; i < groundContactPointGroupList.size(); i++)
         {
            GroundContactPointGroup groundContactGroup = groundContactPointGroupList.get(i);
            groundContactGroup.decideGroundContactPointsInContact();
         }
      }



      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveDecideGroundContactPointsInContact();
      }
   }

   /**
    * This function is called once per simulation tick to update the velocities of all ground contact points.
    * Without this function only those points currently in contact would be updated.
    */
   public void recursiveUpdateAllGroundContactPointVelocities()
   {
      // +++JEP OPTIMIZE

      if (groundContactPointGroupList != null)
      {
         R0_i.set(Ri_0);
         R0_i.transpose();

         for (int i = 0; i < groundContactPointGroupList.size(); i++)
         {
            ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroupList.get(i).getGroundContactPoints();
            for (int y = 0; y < groundContactPoints.size(); y++)
            {
               GroundContactPoint point = groundContactPoints.get(y);
               point.updatePointVelocity(R0_i, owner.link.comOffset, v_i, w_i);
            }
         }

      }

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveUpdateAllGroundContactPointVelocities();
      }
   }

   /**
    * Recurses down the tree ensuring each joint has reasonable accelerations returning false on the first joint to fail.
    *
    * @return Indicates whether or not the joints underwent reasonable accelerations.
    */
   protected boolean verifyReasonableAccelerations()
   {
      if (!jointDependentVerifyReasonableAccelerations())
         return false;

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         if (!child.physics.verifyReasonableAccelerations())
            return false;
      }

      return true;
   }

   public boolean verifySetupProperly(double epsilon)
   {
      if (owner.parentJoint != null)
      {
         Vector3D parentCoMOffset = new Vector3D();
         owner.parentJoint.getLink().getComOffset(parentCoMOffset);
         Vector3D jointOffset = new Vector3D();
         owner.getOffset(jointOffset);

         jointOffset.sub(parentCoMOffset);
         jointOffset.sub(r_in);

         double length = jointOffset.length();
         if (length > epsilon) return false;
      }

      for (Joint childJoint : owner.childrenJoints)
      {
         boolean childSetupProperly = childJoint.physics.verifySetupProperly(epsilon);
         if (!childSetupProperly) return false;
      }

      return true;
   }

   /**
    * Adds the specified KinematicPoint to this joint.  These points allow external forces
    * and effects to be applied while also providing a means to monitor position and velocity.
    * Currently the only implementation internal to SCS is the ExternalForcePoint.
    *
    * @param point KinematicPoint to be added.
    * @see KinematicPoint KinematicPoint
    */
   public void addKinematicPoint(KinematicPoint point)
   {
      if (kinematicPoints == null)
      {
         kinematicPoints = new ArrayList<KinematicPoint>();
      }

      kinematicPoints.add(point);
      point.setParentJoint(owner);
   }

   /**
    * Adds the specified ExternalForcePoint.  These points allow forces to be applied to particular joints
    * allowing the creation of certain mechanical structures such as four-bar-linkages.  See the tutorial for
    * further details.
    *
    * @param point ExternalForcePoint
    * @see ExternalForcePoint ExternalForcePoint
    */
   public void addExternalForcePoint(ExternalForcePoint point)
   {
      if (kinematicPoints == null)
      {
         kinematicPoints = new ArrayList<KinematicPoint>();
      }

      if (externalForcePoints == null)
      {
         externalForcePoints = new ArrayList<ExternalForcePoint>();
      }

      kinematicPoints.add(point);    // Add it to both the external force points, and kinematic points, since it is both.
      externalForcePoints.add(point);
      point.setParentJoint(owner);
   }

   public void addExternalTorque(ExternalTorque torque)
   {
      if (externalTorques == null)
      {
         externalTorques = new ArrayList<ExternalTorque>();
      }

      externalTorques.add(torque);
      torque.setParentJoint(owner);
   }

   public void removeExternalForcePoint( ExternalForcePoint point ) {
      if( !kinematicPoints.remove(point) )
         throw new RuntimeException("Removing point which is not in the kinematics list!");

      if( !externalForcePoints.remove(point) )
         throw new RuntimeException("Removing point which is not in the external force list!");
   }

   public void removeExternalTorque( ExternalTorque point ) {
      if( !externalTorques.remove(point) )
         throw new RuntimeException("Removing torque which is not in the torque list!");

   }

   /**
    * Returns a list of the kinematic points associated with this joint. (Added by Stelian).
    *
    * @param list ArrayList to which the points are added.
    * @see KinematicPoint KinematicPoint
    */
   public void getKinematicPoints(ArrayList<KinematicPoint> list)
   {
      if (kinematicPoints != null)
         list.addAll(this.kinematicPoints);
   }



   /**
    * Recurse over the children of this joint and add their KinematicPoints to the
    * provided ArrayList.  This list includes both KinematicPoints and ExternalForcePoints
    * as the latter is a child of the former.
    *
    * @param list ArrayList to which the points are added.
    * @see KinematicPoint KinematicPoint
    */
   protected void recursiveGetKinematicPoints(ArrayList<KinematicPoint> list)
   {
      if (kinematicPoints != null)
         list.addAll(this.kinematicPoints);

      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveGetKinematicPoints(list);
      }
   }

   /**
    * Recurse over the children of this joint and add their ExternalForcePoints to the
    * provided ArrayList.
    *
    * @param list ArrayList to which the points are added.
    * @see ExternalForcePoint ExternalForcePoint
    */
   protected void recursiveGetExternalForcePoints(ArrayList<ExternalForcePoint> list)
   {
      list.addAll(this.externalForcePoints);

      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveGetExternalForcePoints(list);
      }
   }

   public ArrayList<ExternalForcePoint> getExternalForcePoints()
   {
      return externalForcePoints;
   }

   public ExternalForcePoint getExternalForcePoint(String name)
   {
      if (externalForcePoints == null) 
         return null;

      for (int i=0; i<externalForcePoints.size(); i++)
      {
         ExternalForcePoint externalForcePoint = externalForcePoints.get(i);
         if (externalForcePoint.getName().equals(name)) 
            return externalForcePoint;
      }

      return null;
   }

   /**
    * Recurse over the children of this joint and add their GroundContactPoints to the
    * provided ArrayList.
    *
    * @param list ArrayList to which the points are added.
    * @see GroundContactPoint GroundContactPoint
    */
   public void recursiveGetGroundContactPoints(int groundContactGroupIdentifier, ArrayList<GroundContactPoint> list)
   {
      if (groundContactPointGroups != null)
      {
         list.addAll(groundContactPointGroups.get(groundContactGroupIdentifier).getGroundContactPoints());
      }

      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveGetGroundContactPoints(groundContactGroupIdentifier, list);
      }
   }


   public void recursiveGetAllGroundContactPointsGroupedByJoint(ArrayList<ArrayList<GroundContactPoint>> listOfLists)
   {
      if (groundContactPointGroupList != null)
      {
         ArrayList<GroundContactPoint> listForThisJoint = new ArrayList<GroundContactPoint>();

         for (int i = 0; i < groundContactPointGroupList.size(); i++)
         {
            ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroupList.get(i).getGroundContactPoints();
            listForThisJoint.addAll(groundContactPoints);
         }

         if (!listForThisJoint.isEmpty())
         {
            listOfLists.add(listForThisJoint);
         }
      }

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveGetAllGroundContactPointsGroupedByJoint(listOfLists);
      }

   }

   public void recursiveGetAllGroundContactPoints(ArrayList<GroundContactPoint> list)
   {
      if (groundContactPointGroupList != null)
      {
         for (int i = 0; i < groundContactPointGroupList.size(); i++)
         {
            ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroupList.get(i).getGroundContactPoints();
            list.addAll(groundContactPoints);
         }
      }

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveGetAllGroundContactPoints(list);
      }
   }

   public void recursiveGetAllExternalForcePoints(ArrayList<ExternalForcePoint> list)
   {
      if (externalForcePoints != null)
         list.addAll(externalForcePoints);


      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveGetAllExternalForcePoints(list);
      }
   }

   public void recursiveGetAllKinematicPoints(ArrayList<KinematicPoint> list)
   {
      if (kinematicPoints != null)
         list.addAll(kinematicPoints);

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveGetAllKinematicPoints(list);
      }
   }

   /**
    * Adds the specified GroundContactPoint to this joint.  These points allow ground contact modeling to occur
    * which provides a means of robot ground interaction.  For further examples see the tutorial.
    *
    * @param point GroundContactPoint
    * @see GroundContactPoint GroundContactPoint
    * @see GroundContactModel GroundContactModel
    */
   public void addGroundContactPoint(GroundContactPoint point)
   {
      this.addGroundContactPoint(0, point);
   }

   public void addGroundContactPoint(int groupIdentifier, GroundContactPoint point)
   {
      if (groundContactPointGroups == null)
         groundContactPointGroups = new LinkedHashMap<Integer, GroundContactPointGroup>();

      if (groundContactPointGroupList == null)
      {
         groundContactPointGroupList = new ArrayList<GroundContactPointGroup>();
      }

      GroundContactPointGroup groundContactPointGroup = groundContactPointGroups.get(groupIdentifier);
      if (groundContactPointGroup == null)
      {
         groundContactPointGroup = new GroundContactPointGroup();
         groundContactPointGroups.put(groupIdentifier, groundContactPointGroup);
         groundContactPointGroupList.add(groundContactPointGroup);
      }

      groundContactPointGroup.addGroundContactPoint(point);


      point.setParentJoint(owner);
   }

   public GroundContactPointGroup getGroundContactPointGroup()
   {
      return getGroundContactPointGroup(0);
   }

   public GroundContactPointGroup getGroundContactPointGroup(int identifier)
   {
      if (groundContactPointGroups == null)
         return null;

      return groundContactPointGroups.get(identifier);
   }

   public void addJointWrenchSensor(JointWrenchSensor jointWrenchSensor)
   {
      if (this.jointWrenchSensor != null) throw new RuntimeException("Already have a JointWrenchSensor!");

      this.jointWrenchSensor = jointWrenchSensor;
      tempJointWrenchVector = new SpatialVector();
      tempVectorForWrenchTranslation = new Vector3D();
      tempOffsetForWrenchTranslation = new Vector3D();
   }

   public JointWrenchSensor getJointWrenchSensor()
   {
      return jointWrenchSensor;
   }

   public Vector3D getAngularVelocity()
   {
      return w_i;
   }

   public void freezeFrame()
   {
      this.doFreezeFrame = true;
   }

   public boolean freezeNextFrame()
   {
      return doFreezeFrame;
   }

   public void resetFreezeFrame()
   {
      doFreezeFrame = false;
   }

   public Vector3D getUnitVector()
   {
      return u_i;
   }

   public void getJointAxis(Vector3D axisToPack)
   {
      if (u_i != null)
         axisToPack.set(u_i);
   }

   /**
    * Calculates the rotation matrix between the previous and current joint space.  This method is abstract
    * as different joint types have different means of conversion.  Once calculated the value is stored in the
    * parameter Rh_i.
    *
    * @param Rh_i Matrix3d in which the rotation between from the previous joint space to the current will be stored.
    */
   protected abstract void jointDependentSetAndGetRotation(RotationMatrix Rh_i);

   /**
    * The first featherstone pass handles velocity and position updates as it recurses down the tree.  The primary method does the majority
    * of these calculations however joint torque has a significant impact on both forms of velocity.  Different forms of joints
    * have differing torque conditions therefore each must implement its own method to handle these effects.
    */
   protected abstract void jointDependentFeatherstonePassOne();

   /**
    * Updates the value of d_i based on the current coordinate system.  d_i represents the distance between the inbound joint and
    * its link's center of mass.  Different joint types have varying means of calculating this value, therefore each must
    * implement its own method to handle these effects.
    */
   protected abstract void jointDependentSet_d_i();

   /**
    * The primary function of the second featherstone pass is the calculation of the spatial articulated inertia and
    * spatial articulated zero-acceleration force of link i.  During this process the coriolis forces and spatial joint axis
    * also must be recalculated, however, these values are dependent on joint implementation and type.
    *
    * @param w_h The angular velocity of the previous link in this links coordinate system.  This value is necessary for the
    * calculation of coriolis forces.
    */
   protected abstract void jointDependentFeatherstonePassTwo(Vector3D w_h);

   // protected abstract void jointDependentComputeExternalForceR(Vector3d point_offset, Vector3d comOffset, Vector3d externalForceR);

   /**
    * The fourth and final featherstone pass travels back down the tree calculating joint accelerations based on the data gathered
    * by the preceding three passes.  In order to use the Runge-Kutta (RK4) method to calculate future values this data must be stored
    * over four sets of featherstone passes.  The joint dependant part of the fourth pass stores these values along with the pass number.
    *
    * @param Q Joint acceleration based on the preceding three passes.
    * @param passNumber Number of times dynamics have been calculated.  Dynamics must be calculated four times in total, not to be confused
    * with the four featherstone passes involved in each calculation.
    */
   protected abstract void jointDependentFeatherstonePassFour(double Q, int passNumber);

   /**
    * If a joint is specified as non dynamic the second and third featherstone passes are skipped when dynamics are calculated.  However,
    * Runge-Kutta still requires four k values which are stored by this method.
    *
    * @param passNumber Current pass number of doDynamics.  There are a total of 4 passes numbered 0 - 3.
    */
   protected abstract void jointDependentRecordK(int passNumber);

   /**
    * This function ensures that the joint has not undergone an unreasonable acceleration when dynamics were calculated.  If
    * accelerations are deemed unreasonable the robot
    * Various joint implementations have different definitions of unreasonble and as such implement this method independently.
    *
    * @return Indicates whether or not joint accelerations are reasonable.
    */
   protected abstract boolean jointDependentVerifyReasonableAccelerations();

   // protected abstract void jointDependentRecordK(int passNumber);

   /**
    * This function handles joint dependent velocity changes as a result of collisions and external forces.  Each joint type
    * must implement its own variant of this function.
    *
    * @param delta_qd Change in joint velocity.
    */
   protected abstract void jointDependentChangeVelocity(double delta_qd);

   @Override
   public String toString() {

      StringBuffer retBuffer = new StringBuffer();

      retBuffer.append("   joint axis vector: " + u_i + "\n");

      if ((this.kinematicPoints != null) && (this.kinematicPoints.size() > 0))
      {
         retBuffer.append("\n");
         retBuffer.append("    Kinematic Points: \n");

         for (int i = 0; i < kinematicPoints.size(); i++)
         {
            KinematicPoint kp = kinematicPoints.get(i);
            retBuffer.append("      " + kp.getName());

            if (i != kinematicPoints.size() - 1)
               retBuffer.append("\n");
         }
      }

      if ((this.externalForcePoints != null) && (this.externalForcePoints.size() > 0))
      {
         retBuffer.append("\n");
         retBuffer.append("    External Force Points: \n");

         for (int i = 0; i < externalForcePoints.size(); i++)
         {
            ExternalForcePoint ef = externalForcePoints.get(i);
            retBuffer.append("      " + ef.getName());
            if (i != kinematicPoints.size() - 1)
               retBuffer.append("\n");
         }
      }

      return retBuffer.toString();
   }

   protected void computeAndSetWrenchAtJoint()
   {
      //Using Mirtich Equation 4.24:
      tempJointWrenchVector.set(a_hat_i);

      I_hat_i.multiply(tempJointWrenchVector);
      tempJointWrenchVector.add(Z_hat_i);

      // This is at the center of mass. Now need to resolve it to the joint.
      jointWrenchSensor.getOffsetFromJoint(tempOffsetForWrenchTranslation);
      tempOffsetForWrenchTranslation.scale(-1.0);
      tempOffsetForWrenchTranslation.add(d_i);

      tempVectorForWrenchTranslation.cross(tempOffsetForWrenchTranslation, tempJointWrenchVector.top);
      tempJointWrenchVector.bottom.add(tempVectorForWrenchTranslation);

      // Negate since we want to measure the reaction wrench, not the applied wrench.
      tempJointWrenchVector.scale(-1.0);

      this.jointWrenchSensor.setWrench(tempJointWrenchVector);
   }


}
