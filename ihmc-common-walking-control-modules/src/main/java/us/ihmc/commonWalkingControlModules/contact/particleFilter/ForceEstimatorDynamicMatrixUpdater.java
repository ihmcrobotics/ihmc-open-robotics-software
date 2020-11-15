package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;

public interface ForceEstimatorDynamicMatrixUpdater
{
   /**
    * Sets matrices of the manipulator equation for the estimated system: M * qdd + C * qd + g = tau
    *
    * @param massMatrixToUpdate M in the above equation
    * @param coriolisGravityMatrixToUpdate (C * qd + g) in the above equation
    * @param tauToUpdate tau in the above equation
    */
   void update(DMatrixRMaj massMatrixToUpdate, DMatrixRMaj coriolisGravityMatrixToUpdate, DMatrixRMaj tauToUpdate);
}
