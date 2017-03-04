package us.ihmc.simulationconstructionset.physics.engine.featherstone;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.RigidJoint;


public class RigidJointPhysics extends JointPhysics<RigidJoint>
{
   public RigidJointPhysics(RigidJoint owner)
   {
      super(owner);
   }

   @Override
   protected void jointDependentChangeVelocity(double delta_qd)
   {
   }

   @Override
   protected void jointDependentSetAndGetRotation(RotationMatrix Rh_i)
   {
      System.out.println("RigidJoint jointDependentSetAndGetRotation");
      Rh_i.set(owner.getRigidRotation());
      Rh_i.invert();
   }

   @Override
   protected void jointDependentFeatherstonePassOne()
   {
      // No torques:
      Q_i = 0.0;

      // Ri_0.transform(w_i);  // w and wd not in world coords.  Only x,y,z are
      System.out.println(Ri_h);
      System.out.println(v_i);
      Ri_h.transform(v_i);
   }

   @Override
   protected void jointDependentSet_d_i()
   {
      d_i.set(owner.getRigidTranslation());
      owner.getRigidRotation().inverseTransform(d_i);
      d_i.add(owner.getLink().getComOffset());
   }

   @Override
   protected void jointDependentFeatherstonePassTwo(Vector3D w_h)
   {
      //TODO: Do we need this at all for RigidJoints?
      s_hat_i.top.set(u_i);
      s_hat_i.bottom.cross(u_i, d_i);

      // System.out.print(this.name + ":   " );System.out.println(s_hat_i);
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
