package us.ihmc.robotics.physics;

import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;

public interface ImpulseBasedConstraintCalculator
{
   void initialize(double dt);

   void updateInertia(List<? extends RigidBodyBasics> rigidBodyTargets, List<? extends JointBasics> jointTargets);

   default void computeImpulse(double dt)
   {
      updateImpulse(dt, 1.0, true);
   }

   void updateImpulse(double dt, double alpha, boolean ignoreOtherImpulses);

   default void updateTwistModifiers()
   {
      
   }

   default void finalizeImpulse()
   {

   }

   double getImpulseUpdate();

   double getVelocityUpdate();

   boolean isConstraintActive();

   default void setExternalTwistModifiers(RigidBodyTwistProvider externalRigidBodyTwistModifier, JointStateProvider externalJointTwistModifier)
   {
      if (externalJointTwistModifier.getState() != JointStateType.VELOCITY)
         throw new IllegalArgumentException("Unexpect joint state providers, expected: VELOCITY, was " + externalJointTwistModifier.getState());

      setExternalTwistModifier(externalRigidBodyTwistModifier);
      setExternalTwistModifier(externalJointTwistModifier);
   }

   default void setExternalTwistModifier(RigidBodyTwistProvider externalRigidBodyTwistModifier)
   {

   }

   default void setExternalTwistModifier(JointStateProvider externalJointTwistModifier)
   {

   }

   int getNumberOfRobotsInvolved();

   RigidBodyTwistProvider getRigidBodyTwistChangeProvider(int index);

   JointStateProvider getJointTwistChangeProvider(int index);

   RigidBodyBasics getRootBody(int index);

   DMatrixRMaj getJointVelocityChange(int index);

   List<? extends RigidBodyBasics> getRigidBodyTargets();

   List<? extends JointBasics> getJointTargets();
}
