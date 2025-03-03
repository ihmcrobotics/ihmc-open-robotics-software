package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.providers.BooleanProvider;

public class OrientationCalculator
{
   private final RigidBodyBasics rigidBody;
   private final ReferenceFrame midFeetZUpFrame;
   private final ReferenceFrame centerOfMassFrame;
   private final FullHumanoidRobotModel fullRobotModel;

   private final BooleanProvider isUpperBodyLoadBearing;
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FramePose3D tempPose = new FramePose3D();
   private final FrameQuaternion desiredOrientation = new FrameQuaternion();

   public OrientationCalculator(RigidBodyBasics rigidBody,
                                ReferenceFrame midFeetZUpFrame,
                                ReferenceFrame centerOfMassFrame,
                                FullHumanoidRobotModel fullRobotModel,
                                BooleanProvider isUpperBodyLoadBearing)
   {
      this.rigidBody = rigidBody;
      this.midFeetZUpFrame = midFeetZUpFrame;
      this.centerOfMassFrame = centerOfMassFrame;
      this.fullRobotModel = fullRobotModel;
      this.isUpperBodyLoadBearing = isUpperBodyLoadBearing;
   }

   public void initialize()
   {
      if (isUpperBodyLoadBearing.getValue())
      {
         update();
      }
      else
      {
         tempPose.setToZero(rigidBody.getBodyFixedFrame());
         tempPose.changeFrame(ReferenceFrame.getWorldFrame());
         desiredOrientation.setToYawOrientation(tempPose.getYaw());
      }
   }

   public void update()
   {
      if (isUpperBodyLoadBearing.getValue())
      {
         // Update multi-contact
         tempPoint.setToZero(centerOfMassFrame);
         tempPoint.changeFrame(midFeetZUpFrame);
         double comX = tempPoint.getX();

         tempPoint.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
         tempPoint.changeFrame(midFeetZUpFrame);
         double pelvisZ = tempPoint.getZ();

         double desiredPitch = 0.5 * Math.PI - Math.atan2(pelvisZ, comX);
         desiredOrientation.setYawPitchRollIncludingFrame(midFeetZUpFrame, 0.0, desiredPitch, 0.0);
         desiredOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   public FrameQuaternion getDesiredOrientation()
   {
      return desiredOrientation;
   }
}
