package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Arrays;

public class FilterBasedExternalTorqueEstimator implements ExternalTorqueEstimator
{
   private static final double defaultFilterAlpha = 0.05;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final double dt;
   private final JointBasics[] joints;
   private final int dofs;
   private final DMatrixRMaj M;
   private final DMatrixRMaj qdd;
   private final DMatrixRMaj CqG;
   private final DMatrixRMaj tau;

   private final DMatrixRMaj tauExt;
   private final DMatrixRMaj tauExtFiltered;

   private final ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater;

   public FilterBasedExternalTorqueEstimator(JointBasics[] joints,
                                             double dt,
                                             ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater,
                                             YoRegistry parentRegistry)
   {
      this.joints = joints;
      this.dt = dt;
      this.dynamicMatrixUpdater = dynamicMatrixUpdater;
      this.dofs = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      this.M = new DMatrixRMaj(dofs, dofs);
      this.qdd = new DMatrixRMaj(dofs, 1);
      this.CqG = new DMatrixRMaj(dofs, 1);
      this.tau = new DMatrixRMaj(dofs, 1);
      this.tauExt = new DMatrixRMaj(dofs, 1);

      this.tauExtFiltered = new DMatrixRMaj(dofs, 1);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   @Override
   public DMatrixRMaj getObservedExternalJointTorque()
   {
      return tauExtFiltered;
   }

   @Override
   public void initialize()
   {
//      CommonOps_DDRM.fill(tauExtFiltered, 0.0);
   }

   @Override
   public void doControl()
   {
      dynamicMatrixUpdater.update(M, CqG, tau, qdd);
      CommonOps_DDRM.mult(M, qdd, tauExt);
      CommonOps_DDRM.addEquals(tauExt, CqG);
      CommonOps_DDRM.subtractEquals(tauExt, tau);

      CommonOps_DDRM.scale(defaultFilterAlpha, tauExt);
      CommonOps_DDRM.scale(1.0 - defaultFilterAlpha, tauExtFiltered);
      CommonOps_DDRM.addEquals(tauExtFiltered, tauExt);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void setEstimatorGain(double gain)
   {

   }
}
