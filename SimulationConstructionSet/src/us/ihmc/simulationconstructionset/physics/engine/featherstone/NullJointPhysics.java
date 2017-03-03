package us.ihmc.simulationconstructionset.physics.engine.featherstone;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.NullJoint;


public class NullJointPhysics extends JointPhysics<NullJoint>
{
   public NullJointPhysics(NullJoint owner)
   {
      super(owner);
   }

   @Override
   protected void jointDependentChangeVelocity(double delta_qd)
   {
//    qd.val += delta_qd;
   }

   /*
    * protected void jointDependentSetAndGetRotation(Matrix3d Rh_i)
    * {
    *  Rh_i.setIdentity();  // We probably can rely on Rh_i not changing its 1 and 0 elements but let's just be safe.
    *
    *  double cosQ = Math.cos(q.getDoubleValue()), sinQ = Math.sin(q.getDoubleValue());
    *
    *  if (this.axis == Axis.X) {Rh_i.setElement(1,1,cosQ);Rh_i.setElement(2,2,cosQ);Rh_i.setElement(1,2,-sinQ);Rh_i.setElement(2,1,sinQ);}
    *  else if (this.axis == Axis.Y) {Rh_i.setElement(0,0,cosQ);Rh_i.setElement(2,2,cosQ);Rh_i.setElement(0,2,sinQ);Rh_i.setElement(2,0,-sinQ);}
    *  else if (this.axis == Axis.Z) {Rh_i.setElement(0,0,cosQ);Rh_i.setElement(1,1,cosQ);Rh_i.setElement(0,1,-sinQ);Rh_i.setElement(1,0,sinQ);}
    *
    * }
    */

   @Override
   protected void jointDependentSetAndGetRotation(RotationMatrix Rh_i)
   {
      Rh_i.setIdentity();
   }

   @Override
   protected void jointDependentFeatherstonePassOne()
   {
      // Torque Limits
      Q_i = 0.0;

      // w_i <- w_i + q_i_dot u_i
//    w_i.x = w_i.x + qd.val * u_i.x; w_i.y = w_i.y + qd.val * u_i.y; w_i.z = w_i.z + qd.val * u_i.z;

      // v_i <- v_i + q_i_dot (u_i X d_i)

//    v_i.x = v_i.x + qd.val * u_iXd_i.x;  v_i.y = v_i.y + qd.val * u_iXd_i.y;  v_i.z = v_i.z + qd.val * u_iXd_i.z;

   }

   @Override
   protected void jointDependentSet_d_i()
   {
      d_i.set(owner.getLink().getComOffset());
   }

// private Vector3d vel_iXd_i = new Vector3d();
// private Vector3d w_hXr_i = new Vector3d();
// private Vector3d temp1 = new Vector3d(), temp2 = new Vector3d(), temp3 = new Vector3d();
// private Vector3d vel_i = new Vector3d();  // vel_i is the vector velocity of joint i (vel_i = q_dot_i * u_i)

   @Override
   protected void jointDependentFeatherstonePassTwo(Vector3D w_h)
   {
      // Coriolis Forces:

//    vel_i.set(u_i);
//    vel_i.scale(0.0);

//    c_hat_i.top.cross(w_h, vel_i);
//    vel_iXd_i.cross(vel_i, d_i);

//    w_hXr_i.cross(w_h, r_i);

//    temp1.cross(w_h, w_hXr_i);
//    temp2.cross(w_h, vel_iXd_i);
//    temp3.cross(vel_i, vel_iXd_i);
//
//    temp2.scale(2.0);

//    c_hat_i.bottom.add(temp1, temp2);
//    c_hat_i.bottom.add(temp3);

      // Spatial Joint axis:

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
