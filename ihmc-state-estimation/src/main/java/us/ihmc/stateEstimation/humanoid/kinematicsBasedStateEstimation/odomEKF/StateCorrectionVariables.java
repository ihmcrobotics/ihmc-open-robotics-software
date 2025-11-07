package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.*;

class StateCorrectionVariables
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public final YoFramePoint3D translation;
   public final YoFrameVector3D linearVelocity;
   public final YoFrameVector3D orientation;
   public final YoFrameVector3D accelBias;
   public final YoFrameVector3D gyroBias;

   public StateCorrectionVariables(String prefix, MovingReferenceFrame sensorFrame, YoRegistry registry)
   {
      translation = new YoFramePoint3D(prefix + "TranslationCorrection", worldFrame, registry);
      linearVelocity = new YoFrameVector3D(prefix + "LinearVelocityCorrection", worldFrame, registry);
      orientation = new YoFrameVector3D(prefix + "OrientationCorrection", worldFrame, registry);
      accelBias = new YoFrameVector3D(prefix + "AccelBiasCorrection", sensorFrame, registry);
      gyroBias = new YoFrameVector3D(prefix + "GyroBiasCorrection", sensorFrame, registry);
   }

   public void set(int start, DMatrixRMaj state)
   {
      translation.set(start + errorTranslationIndex, state);
      linearVelocity.set(start + errorLinearVelocityIndex, state);
      orientation.set(start + errorOrientationIndex, state);
      accelBias.set(start + errorAccelBiasIndex, state);
      gyroBias.set(start + errorGyroBiasIndex, state);
   }
}
