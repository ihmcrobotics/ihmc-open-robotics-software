package us.ihmc.simulationconstructionset.physics.engine.featherstone;

import java.util.ArrayList;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.RigidJoint;
import us.ihmc.simulationconstructionset.SpatialVector;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;


public class RigidJointPhysics extends JointPhysics<RigidJoint>
{
   public RigidJointPhysics(RigidJoint owner)
   {
      super(owner);
      
      u_i = null;
      u_iXd_i = null;
      s_hat_i = null;
   }

   @Override
   protected void jointDependentChangeVelocity(double delta_qd)
   {
   }

   @Override
   protected void jointDependentSetAndGetRotation(RotationMatrix Rh_i)
   {
      Rh_i.set(owner.getRigidRotation());
   }

   @Override
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

      //TODO: Diff here with JointPhysics:
//      this.u_iXd_i.cross(u_i, d_i);

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

   @Override
   protected void jointDependentSet_d_i()
   {
      d_i.set(owner.getRigidTranslation());
      owner.getRigidRotation().inverseTransform(d_i);
      d_i.add(owner.getLink().getComOffset());
   }

   @Override
   public void featherstonePassThree()
   {
      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.featherstonePassThree(I_hat_i, Z_hat_i);
      }
   }
   
   @Override
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

      SIM2.set(I_hat_i);
      h_X_hat_i.transformSpatialInertia(SIM2);

      I_hat_h.add(SIM2);

      // Now do Z_hat_h
      sV1.set(c_hat_i);
      I_hat_i.multiply(sV1);

      sV1.add(Z_hat_i);

      h_X_hat_i.transform(sV1);
      Z_hat_h.add(sV1);
   }

   @Override
   public void featherstonePassFour(SpatialVector a_hat_h, int passNumber) throws UnreasonableAccelerationException
   {
      X_hat_h_a_hat_h.set(a_hat_h);
      i_X_hat_h.transform(X_hat_h_a_hat_h);
      I_hat_i.multiply(X_hat_h_a_hat_h);

      this.jointDependentFeatherstonePassFour(0.0, passNumber);

      a_hat_i.set(a_hat_h);
      i_X_hat_h.transform(a_hat_i);
      a_hat_i.add(c_hat_i);

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

   @Override
   protected void jointDependentFeatherstonePassOne()
   {
   }

   private Vector3D w_hXr_i = new Vector3D();
   private Vector3D temp1 = new Vector3D();

   @Override
   protected void jointDependentFeatherstonePassTwo(Vector3D w_h)
   {
      // Coriolis Forces:
      c_hat_i.top.set(0.0, 0.0, 0.0);
      w_hXr_i.cross(w_h, r_i);
      temp1.cross(w_h, w_hXr_i);
      c_hat_i.bottom.set(temp1);
   }

   @Override
   protected void jointDependentFeatherstonePassFour(double Q, int passNumber)
   {
   }

   @Override
   protected void jointDependentRecordK(int passNumber)
   {
   }

   @Override
   public void recursiveEulerIntegrate(double stepSize)
   {
      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveEulerIntegrate(stepSize);
      }
   }

   @Override
   public void recursiveRungeKuttaSum(double stepSize)
   {
      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveRungeKuttaSum(stepSize);
      }
   }

   @Override
   public void recursiveSaveTempState()
   {
      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveSaveTempState();
      }
   }

   @Override
   public void recursiveRestoreTempState()
   {
      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveRestoreTempState();
      }
   }

   @Override
   protected boolean jointDependentVerifyReasonableAccelerations()
   {
      return true;
   }

}
