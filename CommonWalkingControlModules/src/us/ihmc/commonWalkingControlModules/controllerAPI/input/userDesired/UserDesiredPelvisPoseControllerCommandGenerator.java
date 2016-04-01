package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class UserDesiredPelvisPoseControllerCommandGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable userDoPelvisPose = new BooleanYoVariable("userDoPelvisPose", registry);
   private final BooleanYoVariable userUpdateDesiredPelvisPose = new BooleanYoVariable("userUpdateDesiredPelvisPose", registry);
   private final DoubleYoVariable userDesiredPelvisPoseTrajectoryTime = new DoubleYoVariable("userDesiredPelvisPoseTrajectoryTime", registry);
   private final YoFramePose userDesiredPelvisPose;

   private final ReferenceFrame midFeetZUpFrame, pelvisFrame;

   private final FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());

   public UserDesiredPelvisPoseControllerCommandGenerator(final CommandInputManager controllerCommandInputManager,
         final FullHumanoidRobotModel fullRobotModel, CommonHumanoidReferenceFrames commonHumanoidReferenceFrames, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      midFeetZUpFrame = commonHumanoidReferenceFrames.getMidFeetZUpFrame();
      pelvisFrame = commonHumanoidReferenceFrames.getPelvisFrame();
      userDesiredPelvisPose = new YoFramePose("userDesiredPelvisPose", midFeetZUpFrame, registry);

      userUpdateDesiredPelvisPose.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            if (userUpdateDesiredPelvisPose.getBooleanValue())
            {
               framePose.setToZero(pelvisFrame);
               framePose.changeFrame(midFeetZUpFrame);
               userDesiredPelvisPose.set(framePose);
               userUpdateDesiredPelvisPose.set(false);
            }
         }
      });
      
      userDoPelvisPose.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            if (userDoPelvisPose.getBooleanValue())
            {
               userDesiredPelvisPose.getFramePoseIncludingFrame(framePose);
               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               System.out.println("framePose " + framePose);

               PelvisTrajectoryCommand pelvisTrajectoryControllerCommand = new PelvisTrajectoryCommand();

               FrameSE3TrajectoryPoint trajectoryPoint = new FrameSE3TrajectoryPoint(ReferenceFrame.getWorldFrame());
               trajectoryPoint.setTime(userDesiredPelvisPoseTrajectoryTime.getDoubleValue());
               trajectoryPoint.setPosition(framePose.getFramePointCopy());
               trajectoryPoint.setOrientation(framePose.getFrameOrientationCopy());
               trajectoryPoint.setLinearVelocity(new Vector3d());
               trajectoryPoint.setAngularVelocity(new Vector3d());

               pelvisTrajectoryControllerCommand.addTrajectoryPoint(trajectoryPoint);
               System.out.println("Submitting " + pelvisTrajectoryControllerCommand);
               controllerCommandInputManager.submitCommand(pelvisTrajectoryControllerCommand);

               userDoPelvisPose.set(false);
            }
         }
      });

      userDesiredPelvisPoseTrajectoryTime.set(defaultTrajectoryTime);
      parentRegistry.addChild(registry);
   }

}
