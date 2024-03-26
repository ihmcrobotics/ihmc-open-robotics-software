package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

public class KSTInputFilter
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoVector3D bbxSize = new YoVector3D("inputFilterBBXSize", registry);
   private final YoFramePoint3D bbxOffset;
   private final YoFramePoint3D bbxPosition = new YoFramePoint3D("inputFilterBBXPosition", worldFrame, registry);
   private final YoFrameQuaternion bbxOrientation = new YoFrameQuaternion("inputFilterBBXOrientation", worldFrame, registry);
   private final MidFootZUpGroundFrame midFeetZUpFrame;

   public KSTInputFilter(FullHumanoidRobotModel fullRobotModel, KinematicsStreamingToolboxParameters parameters, YoRegistry parentRegistry)
   {
      midFeetZUpFrame = new MidFootZUpGroundFrame("midFeetZUpFrame", fullRobotModel.getSoleFrame(RobotSide.LEFT), fullRobotModel.getSoleFrame(RobotSide.RIGHT));
      if (parameters.getInputFilterBBXSize() != null)
         bbxSize.set(parameters.getInputFilterBBXSize());
      bbxOffset = new YoFramePoint3D("inputFilterBBXCenter", midFeetZUpFrame, registry);
      if (parameters.getInputFilterBBXCenter() != null)
         bbxOffset.set(parameters.getInputFilterBBXCenter());

      parentRegistry.addChild(registry);
   }

   private final Box3D boundingBox = new Box3D();

   public void update()
   {
      midFeetZUpFrame.update();
      bbxPosition.setMatchingFrame(bbxOffset);
      bbxOrientation.setToYawOrientation(midFeetZUpFrame.getTransformToRoot().getRotation().getYaw());
      boundingBox.getPose().set(bbxOrientation, bbxPosition);
      boundingBox.getSize().set(bbxSize);
   }

   public boolean isInputValid(KinematicsToolboxRigidBodyCommand input)
   {
      return boundingBox.isPointInside(input.getDesiredPose().getPosition());
   }
}
