package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.*;
import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.measurementAccelIndex;
import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.measurementContactVelocityIndex;

class MeasurementVariables
{
   public final YoVector3D relativePosition;
   public final YoVector3D relativeOrientationError;
   public final YoVector3D relativeLinearVelocityError;
   public final YoVector3D contactVelocity;
   public final YoVector3D accelMeasure;


   public MeasurementVariables(String prefix, YoRegistry registry)
   {
      relativePosition = new YoVector3D(prefix + "RelativePosition", registry);
      relativeOrientationError = new YoVector3D(prefix + "RelativeOrientationError", registry);
      relativeLinearVelocityError = new YoVector3D(prefix + "RelativeLinearVelocityError", registry);
      contactVelocity = new YoVector3D(prefix + "ContactVelocity", registry);
      accelMeasure = new YoVector3D(prefix + "AccelMeasure", registry);
   }

   public void get(int start, DMatrixRMaj measurementToPack)
   {
      relativePosition.get(start + measurementRelativeTranslationIndex, measurementToPack);
      relativeOrientationError.get(start + measurementRelativeOrientationErrorIndex, measurementToPack);
      relativeLinearVelocityError.get(start + measurementRelativeVelocityIndex, measurementToPack);
      contactVelocity.get(start + measurementContactVelocityIndex, measurementToPack);
      accelMeasure.get(start + measurementAccelIndex, measurementToPack);
   }
}
