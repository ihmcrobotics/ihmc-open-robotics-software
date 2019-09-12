package us.ihmc.atlas;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.stateEstimation.head.carnegie.multisense.MultisenseSLWithMicroStrainHeadPoseEstimator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

import java.io.IOException;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasHeadPoseEstimator extends MultisenseSLWithMicroStrainHeadPoseEstimator
{
   private static final PriorityParameters imuListenerPriority = new PriorityParameters(20);

   private final String simpleName = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(simpleName);

   private final RigidBodyTransform headRigidBodyTransform = new RigidBodyTransform();

   private final YoFrameYawPitchRoll yoFrameYawPitchRoll;
   private final YoFramePoint3D yoPosition;

   public AtlasHeadPoseEstimator(double dt, long microStrainSerialNumber, boolean getRobotConfigurationDataFromNetwork, FullHumanoidRobotModel fullRobotModel,
                                 YoVariableRegistry parentRegistry) throws IOException
   {
      super(fullRobotModel, dt, MultisenseSLWithMicroStrainHeadPoseEstimator.DEFAULT_IMU_TO_MULTISENSE_TRANSFORM, imuListenerPriority, microStrainSerialNumber,
            getRobotConfigurationDataFromNetwork, parentRegistry);

      yoFrameYawPitchRoll = new YoFrameYawPitchRoll(simpleName, ReferenceFrame.getWorldFrame(), registry);
      yoPosition = new YoFramePoint3D(simpleName, ReferenceFrame.getWorldFrame(), registry);

      parentRegistry.addChild(registry);
   }

   public AtlasHeadPoseEstimator(double dt, long microStrainSerialNumber, FullHumanoidRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
         throws IOException
   {
      this(dt, microStrainSerialNumber, false, fullRobotModel, parentRegistry);
   }

   public AtlasHeadPoseEstimator(double dt, long microStrainSerialNumber, AtlasRobotModel atlasRobotModel, YoVariableRegistry parentRegistry) throws IOException
   {
      this(dt, microStrainSerialNumber, true, atlasRobotModel.createFullRobotModel(), parentRegistry);
   }

   @Override
   public void compute()
   {
      super.compute();
      getHeadTransform(headRigidBodyTransform);

      yoFrameYawPitchRoll.set(headRigidBodyTransform.getRotation());
      yoPosition.set(headRigidBodyTransform.getTranslation());
   }
}
