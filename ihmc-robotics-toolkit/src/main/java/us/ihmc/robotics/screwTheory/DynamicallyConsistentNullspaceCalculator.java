package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * @author twan
 *         Date: 4/17/13
 */
public interface DynamicallyConsistentNullspaceCalculator
{
   void reset();

   void addConstraint(RigidBodyBasics body, DMatrixRMaj selectionMatrix);

   void addActuatedJoint(JointBasics joint);

   void compute();

   DMatrixRMaj getDynamicallyConsistentNullspace();

   DMatrixRMaj getSNsBar();
}
