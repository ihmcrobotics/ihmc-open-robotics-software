package us.ihmc.atlas;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.stateEstimation.head.carnegie.multisense.MultisenseSLWithMicroStrainHeadPoseEstimator;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

import java.io.IOException;
import java.io.InputStream;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasHeadPoseEstimator extends MultisenseSLWithMicroStrainHeadPoseEstimator
{
   private static final String PARAMETER_FILE = "/us/ihmc/atlas/parameters/head_pose_estimator.xml";
   private static final PriorityParameters imuListenerPriority = new PriorityParameters(20);

   private final String simpleName = getClass().getSimpleName();

   private final RigidBodyTransform headRigidBodyTransform = new RigidBodyTransform();

   private final YoFrameYawPitchRoll yoFrameYawPitchRoll;
   private final YoFramePoint3D yoPosition;

   public AtlasHeadPoseEstimator(double dt, long microStrainSerialNumber, boolean getRobotConfigurationDataFromNetwork, FullHumanoidRobotModel fullRobotModel)
         throws IOException
   {
      super(fullRobotModel, dt, MultisenseSLWithMicroStrainHeadPoseEstimator.DEFAULT_IMU_TO_MULTISENSE_TRANSFORM, imuListenerPriority, microStrainSerialNumber,
            getRobotConfigurationDataFromNetwork);

      InputStream resourceAsStream = getClass().getResourceAsStream(PARAMETER_FILE);
      //      ParameterLoaderHelper.loadParameters(this, resourceAsStream, getRegistry());
      yoFrameYawPitchRoll = new YoFrameYawPitchRoll(simpleName, ReferenceFrame.getWorldFrame(), getRegistry());
      yoPosition = new YoFramePoint3D(simpleName, ReferenceFrame.getWorldFrame(), getRegistry());
   }

   public AtlasHeadPoseEstimator(double dt, long microStrainSerialNumber, FullHumanoidRobotModel fullRobotModel)
         throws IOException
   {
      this(dt, microStrainSerialNumber, false, fullRobotModel);
   }

   public AtlasHeadPoseEstimator(double dt, long microStrainSerialNumber, AtlasRobotModel atlasRobotModel) throws IOException
   {
      this(dt, microStrainSerialNumber, true, atlasRobotModel.createFullRobotModel());
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
