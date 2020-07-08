package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class ImpulseBasedJointTwistProvider implements JointStateProvider
{
   private final RigidBodyBasics rootBody;
   private int impulseDimension;

   private boolean isImpulseZero = true;
   private final DMatrixRMaj impulse = new DMatrixRMaj(6, 1);
   private final List<JointBasics> joints = new ArrayList<>();
   private final Map<JointBasics, DMatrixRMaj> apparentInertiaMatrixInverseMap = new HashMap<>();

   public ImpulseBasedJointTwistProvider(RigidBodyBasics rootBody)
   {
      this.rootBody = rootBody;
   }

   public void clear(int impulseDimension)
   {
      isImpulseZero = true;
      this.impulseDimension = impulseDimension;
      impulse.reshape(impulseDimension, 1);
      impulse.zero();
      joints.clear();
      apparentInertiaMatrixInverseMap.clear();
   }

   public void addAll(Collection<? extends JointBasics> joints)
   {
      for (JointBasics joint : joints)
      {
         add(joint);
      }
   }

   public void add(JointBasics joint)
   {
      if (MultiBodySystemTools.getRootBody(joint.getPredecessor()) != rootBody)
         return;

      joints.add(joint);
      apparentInertiaMatrixInverseMap.put(joint, new DMatrixRMaj(joint.getDegreesOfFreedom(), impulseDimension));
   }

   public List<JointBasics> getJoints()
   {
      return joints;
   }

   public DMatrixRMaj getApparentInertiaMatrixInverse(JointBasics joint)
   {
      return apparentInertiaMatrixInverseMap.get(joint);
   }

   public void setImpulseToZero()
   {
      isImpulseZero = true;
      impulse.zero();
   }

   public void setImpulse(double impulse)
   {
      isImpulseZero = false;
      this.impulse.set(0, impulse);
   }

   public void setImpulse(DMatrixRMaj impulse)
   {
      isImpulseZero = false;
      this.impulse.set(impulse);
   }

   public void setImpulse(Vector3DReadOnly impulse)
   {
      isImpulseZero = false;
      impulse.get(this.impulse);
   }

   public void setImpulse(Vector3DReadOnly impulseLinear, double impulseAngular)
   {
      isImpulseZero = false;
      impulseLinear.get(this.impulse);
      this.impulse.set(3, 0, impulseAngular);
   }

   @Override
   public JointStateType getState()
   {
      return JointStateType.VELOCITY;
   }

   private final DMatrixRMaj jointTwist = new DMatrixRMaj(6, 1);

   @Override
   public DMatrixRMaj getJointState(JointReadOnly joint)
   {
      if (isImpulseZero)
         return null;

      DMatrixRMaj apparentInertiaMatrixInverse = apparentInertiaMatrixInverseMap.get(joint);

      if (apparentInertiaMatrixInverse == null)
         return null;

      jointTwist.reshape(joint.getDegreesOfFreedom(), 1);
      CommonOps_DDRM.mult(apparentInertiaMatrixInverse, impulse, jointTwist);

      return jointTwist;
   }
}