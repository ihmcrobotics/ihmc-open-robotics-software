package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.*;

class StateVariables
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public final YoFramePoint3D translation;
   public final YoFrameVector3D linearVelocity;
   public final YoFrameQuaternion orientation;
   public final YoFrameVector3D accelBias;
   public final YoFrameVector3D gyroBias;

   public final YoFrameVector3D unbiasedAccel;
   public final YoFrameVector3D unbiasedGyro;

   private final MovingReferenceFrame sensorFrame;

   public StateVariables(String prefix, MovingReferenceFrame sensorFrame, YoRegistry registry)
   {
      this.sensorFrame = sensorFrame;

      translation = new YoFramePoint3D(prefix + "Translation", worldFrame, registry);
      linearVelocity = new YoFrameVector3D(prefix + "LinearVelocity", worldFrame, registry);
      orientation = new YoFrameQuaternion(prefix + "Orientation", worldFrame, registry);
      accelBias = new YoFrameVector3D(prefix + "AccelBias", sensorFrame, registry);
      gyroBias = new YoFrameVector3D(prefix + "GyroBias", sensorFrame, registry);
      unbiasedAccel = new YoFrameVector3D(prefix + "UnbiasedAccel", sensorFrame, registry);
      unbiasedGyro = new YoFrameVector3D(prefix + "UnbiasedGyro", sensorFrame, registry);
   }

   public void initialize()
   {
      this.translation.setFromReferenceFrame(sensorFrame);
      this.orientation.setFromReferenceFrame(sensorFrame);
      linearVelocity.setMatchingFrame(sensorFrame.getTwistOfFrame().getLinearPart());
      accelBias.setToZero();
      gyroBias.setToZero();
   }

   public void set(int start, DMatrixRMaj state, SensedVariables sensedVariables)
   {
      translation.set(start + stateTranslationIndex, state);
      linearVelocity.set(start + stateLinearVelocityIndex, state);
      orientation.set(start + stateOrientationIndex, state);
      accelBias.set(start + stateAccelBiasIndex, state);
      gyroBias.set(start + stateGyroBiasIndex, state);

      if (OdometryKalmanFilter.includeBias)
      {
         unbiasedAccel.sub(sensedVariables.accelMeasurement, accelBias);
         unbiasedGyro.sub(sensedVariables.gyroMeasurement, gyroBias);
      }
      else
      {
         unbiasedAccel.set(sensedVariables.accelMeasurement);
         unbiasedGyro.set(sensedVariables.gyroMeasurement);
      }
   }

   public void get(int start, DMatrixRMaj stateToPack)
   {
      translation.get(start + stateTranslationIndex, stateToPack);
      linearVelocity.get(start + stateLinearVelocityIndex, stateToPack);
      orientation.get(start + stateOrientationIndex, stateToPack);
      accelBias.get(start + stateAccelBiasIndex, stateToPack);
      gyroBias.get(start + stateGyroBiasIndex, stateToPack);
   }
}
