package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class ConstantPelvisRotationalStateUpdater implements PelvisRotationalStateUpdaterInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameOrientation yoRootJointFrameOrientation = new YoFrameOrientation("constantRootJoint", worldFrame, registry);

   private final FloatingInverseDynamicsJoint rootJoint;

   public ConstantPelvisRotationalStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, YoVariableRegistry parentRegistry)
   {
      rootJoint = inverseDynamicsStructure.getRootJoint();
      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      updateRootJointOrientationAndAngularVelocity();
   }

   @Override
   public void initializeForFrozenState()
   {
      updateRootJointOrientationAndAngularVelocity();
   }

   @Override
   public void updateForFrozenState()
   {
      updateRootJointOrientationAndAngularVelocity();
   }

   private final Quaternion rootJointOrientation = new Quaternion();
   private final Twist twistRootBodyRelativeToWorld = new Twist();

   @Override
   public void updateRootJointOrientationAndAngularVelocity()
   {
      yoRootJointFrameOrientation.getQuaternion(rootJointOrientation);
      rootJoint.setRotation(rootJointOrientation);

      rootJoint.getJointTwist(twistRootBodyRelativeToWorld);
      twistRootBodyRelativeToWorld.setToZero();
      rootJoint.setJointTwist(twistRootBodyRelativeToWorld);
   }

   @Override
   public void getEstimatedOrientation(FrameOrientation estimatedOrientation)
   {
      yoRootJointFrameOrientation.getFrameOrientationIncludingFrame(estimatedOrientation);
   }

   @Override
   public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
   {
      estimatedAngularVelocityToPack.setToZero();
   }
}
