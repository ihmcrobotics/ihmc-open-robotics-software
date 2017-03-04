package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.RigidJointPhysics;

/**
 * RigidJoint is a Robot Joint which has no motion. It is used in order to create a transformation from the previous joint.
 * This is useful if you want to have the following joint axes at non pure X, Y, Z axes, for example if you have the zero 
 * positions of robot shoulders canted at 45 degrees, but want the rest of the joints to be straight off of that 45 degrees
 * when their joint angles are 0. You can achieve that without this class, by computing their joint axes, but it is more
 * convenient to use this class instead. 
 *
 */
public class RigidJoint extends Joint
{
   private static final long serialVersionUID = 1341493615657008348L;

   private final Vector3D rigidTranslation = new Vector3D();
   private final RotationMatrix rigidRotation = new RotationMatrix();

   public RigidJoint(String jname, Vector3D offset, Robot rob)
   {
      super(jname, offset, rob, 0);
      physics = new RigidJointPhysics(this);

      //TODO: Need this at all?
      physics.u_i = new Vector3D(1.0, 0.0, 0.0);
   }

   @Override
   protected void update()
   {
      this.jointTransform3D.setTranslation(rigidTranslation);
      this.jointTransform3D.setRotation(rigidRotation);
   }

   public void setRigidTranslation(Vector3D jointTranslation)
   {
      this.rigidTranslation.set(jointTranslation);
   }

   public void setRigidRotation(RotationMatrix jointRotation)
   {
      this.rigidRotation.set(jointRotation);
   }

   public Vector3DReadOnly getRigidTranslation()
   {
      return rigidTranslation;
   }

   public RotationMatrixReadOnly getRigidRotation()
   {
      return rigidRotation;
   }
}
