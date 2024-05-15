package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTInputFilter
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final String namePrefix = "inputFilter";
   private final YoVector3D bbxSize = new YoVector3D(namePrefix + "BBXSize", registry);
   private final YoFramePoint3D bbxOffset;
   private final YoFramePoint3D bbxPosition = new YoFramePoint3D(namePrefix + "BBXPosition", worldFrame, registry);
   private final YoFrameQuaternion bbxOrientation = new YoFrameQuaternion(namePrefix + "BBXOrientation", worldFrame, registry);
   private final MidFootZUpGroundFrame midFeetZUpFrame;

   private final YoBoolean enableBBXFilter = new YoBoolean(namePrefix + "EnableBBXFilter", registry);

   private final YoDouble maxLinearDelta = new YoDouble(namePrefix + "MaxLinearDelta", registry);
   private final YoDouble maxAngularDelta = new YoDouble(namePrefix + "MaxAngularDelta", registry);
   private final YoDouble maxLinearVelocity = new YoDouble(namePrefix + "MaxLinearVelocity", registry);
   private final YoDouble maxAngularVelocity = new YoDouble(namePrefix + "MaxAngularVelocity", registry);

   public KSTInputFilter(FullHumanoidRobotModel fullRobotModel, KinematicsStreamingToolboxParameters parameters, YoRegistry parentRegistry)
   {
      midFeetZUpFrame = new MidFootZUpGroundFrame("midFeetZUpFrame", fullRobotModel.getSoleFrame(RobotSide.LEFT), fullRobotModel.getSoleFrame(RobotSide.RIGHT));
      enableBBXFilter.set(parameters.isUseBBXInputFilter());
      if (parameters.getInputFilterBBXSize() != null)
         bbxSize.set(parameters.getInputFilterBBXSize());
      bbxOffset = new YoFramePoint3D(namePrefix + "BBXCenter", midFeetZUpFrame, registry);
      if (parameters.getInputFilterBBXCenter() != null)
         bbxOffset.set(parameters.getInputFilterBBXCenter());

      maxLinearDelta.set(parameters.getInputFilterMaxLinearDelta());
      maxAngularDelta.set(parameters.getInputFilterMaxAngularDelta());
      maxLinearVelocity.set(parameters.getInputFilterMaxLinearVelocity());
      maxAngularVelocity.set(parameters.getInputFilterMaxAngularVelocity());

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

   public boolean isInputValid(KinematicsToolboxCenterOfMassCommand input, KinematicsToolboxCenterOfMassCommand previousInput)
   {
      if (enableBBXFilter.getValue() && !boundingBox.isPointInside(input.getDesiredPosition()))
      {
         LogTools.warn("Invalid CoM input. Input outside bounding box");
         return false;
      }

      if (previousInput != null)
      {
         double linearDelta = input.getDesiredPosition().distance(previousInput.getDesiredPosition());
         if (linearDelta > maxLinearDelta.getValue())
         {
            LogTools.warn("Invalid CoM input. Linear delta {} > maximum delta {}", linearDelta, maxLinearDelta.getValue());
            return false;
         }
      }

      if (input.getHasDesiredVelocity())
      {
         double linearVelocity = input.getDesiredVelocity().norm();
         if (linearVelocity > maxLinearVelocity.getValue())
            return false;
      }

      return true;
   }

   public boolean isInputValid(KinematicsToolboxRigidBodyCommand input, KinematicsToolboxRigidBodyCommand previousInput)
   {
      if (enableBBXFilter.getValue() && !boundingBox.isPointInside(input.getDesiredPose().getPosition()))
      {
         LogTools.warn("Invalid rigid body input. Input outside bounding box");
         return false;
      }

      if (previousInput != null)
      {
         double linearDelta = input.getDesiredPose().getPositionDistance(previousInput.getDesiredPose());
         if (linearDelta > maxLinearDelta.getValue())
         {
            LogTools.warn("Invalid rigid body input. Linear delta {} > maximum delta {}", linearDelta, maxLinearDelta.getValue());
            return false;
         }
         double angularDelta = input.getDesiredPose().getOrientationDistance(previousInput.getDesiredPose());
         if (angularDelta > maxAngularDelta.getValue())
         {
            LogTools.warn("Invalid rigid body input. Angular delta {} > maximum delta {}", angularDelta, maxAngularDelta.getValue());
            return false;
         }
      }

      if (input.getHasDesiredVelocity())
      {
         double linearVelocity = input.getDesiredVelocity().getLinearPart().norm();
         if (linearVelocity > maxLinearVelocity.getValue())
            return false;
         double angularVelocity = input.getDesiredVelocity().getAngularPart().norm();
         if (angularVelocity > maxAngularVelocity.getValue())
            return false;
      }

      return true;
   }
}
