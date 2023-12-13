package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ConstantPelvisRotationalStateUpdater implements PelvisRotationalStateUpdaterInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoFrameYawPitchRoll yoRootJointFrameOrientation = new YoFrameYawPitchRoll("constantRootJoint", worldFrame, registry);

   private final FloatingJointBasics rootJoint;

   public ConstantPelvisRotationalStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, YoRegistry parentRegistry)
   {
      rootJoint = inverseDynamicsStructure.getRootJoint();
      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      updateRootJointOrientationAndAngularVelocity();
   }

   private final Quaternion rootJointOrientation = new Quaternion();
   private final Twist twistRootBodyRelativeToWorld = new Twist();

   @Override
   public void updateRootJointOrientationAndAngularVelocity()
   {
      rootJointOrientation.set(yoRootJointFrameOrientation);
      rootJoint.setJointOrientation(rootJointOrientation);

      twistRootBodyRelativeToWorld.setIncludingFrame(rootJoint.getJointTwist());
      twistRootBodyRelativeToWorld.setToZero();
      rootJoint.setJointTwist(twistRootBodyRelativeToWorld);
   }

   @Override
   public FrameOrientation3DReadOnly getEstimatedOrientation()
   {
      return yoRootJointFrameOrientation;
   }

   @Override
   public FrameVector3DReadOnly getEstimatedAngularVelocity()
   {
      return twistRootBodyRelativeToWorld.getAngularPart();
   }
}
