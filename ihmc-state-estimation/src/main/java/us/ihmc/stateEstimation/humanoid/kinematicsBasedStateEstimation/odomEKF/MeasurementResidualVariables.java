package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.*;

class MeasurementResidualVariables
{
   // State variables
   public final YoVector3D footPositionResidual;
   public final YoVector3D footOrientationResidual;
   public final YoVector3D linearVelocityResidual;
   public final YoVector3D contactVelocityResidual;
   public final YoVector3D contactAccelResidual;

   public MeasurementResidualVariables(String prefix, YoRegistry registry)
   {
      footPositionResidual = new YoVector3D(prefix + "PositionResidual", registry);
      footOrientationResidual = new YoVector3D(prefix + "OrientationResidual", registry);
      linearVelocityResidual = new YoVector3D(prefix + "LinearVelocityResidual", registry);
      contactVelocityResidual = new YoVector3D(prefix + "ContactVelocityResidual", registry);
      contactAccelResidual = new YoVector3D(prefix + "ContactAccelResidual", registry);
   }

   public void set(int start, DMatrixRMaj residual)
   {
      footPositionResidual.set(start + measurementRelativeTranslationIndex, residual);
      footOrientationResidual.set(start + measurementRelativeOrientationErrorIndex, residual);
      linearVelocityResidual.set(start + measurementRelativeVelocityIndex, residual);
      contactVelocityResidual.set(start + measurementContactVelocityIndex, residual);
      contactAccelResidual.set(start + measurementAccelIndex, residual);
   }
}
