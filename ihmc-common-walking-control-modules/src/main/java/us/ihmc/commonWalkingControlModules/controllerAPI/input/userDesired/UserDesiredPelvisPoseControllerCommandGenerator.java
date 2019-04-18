package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class UserDesiredPelvisPoseControllerCommandGenerator
{
   private static final boolean DEBUG = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean userDoPelvisPose = new YoBoolean("userDoPelvisPose", registry);
   private final YoBoolean userStreamPelvisPose = new YoBoolean("userStreamPelvisPose", registry);
   private final YoBoolean userStreamPelvisOrientation = new YoBoolean("userStreamPelvisOrientation", registry);
   private final YoBoolean userUpdateDesiredPelvisPose = new YoBoolean("userUpdateDesiredPelvisPose", registry);
   private final YoDouble userDesiredPelvisPoseTrajectoryTime = new YoDouble("userDesiredPelvisPoseTrajectoryTime", registry);
   private final YoFramePoseUsingYawPitchRoll userDesiredPelvisPose;

   private final ReferenceFrame midFeetZUpFrame, pelvisFrame;

   private final FramePose3D framePose = new FramePose3D(worldFrame);

   private final CommandInputManager controllerCommandInputManager;

   public UserDesiredPelvisPoseControllerCommandGenerator(final CommandInputManager controllerCommandInputManager, final FullHumanoidRobotModel fullRobotModel,
         CommonHumanoidReferenceFrames commonHumanoidReferenceFrames, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      this.controllerCommandInputManager = controllerCommandInputManager;
      midFeetZUpFrame = commonHumanoidReferenceFrames.getMidFeetZUpFrame();
      pelvisFrame = commonHumanoidReferenceFrames.getPelvisFrame();
      userDesiredPelvisPose = new YoFramePoseUsingYawPitchRoll("userDesiredPelvisPose", midFeetZUpFrame, registry);

      userUpdateDesiredPelvisPose.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
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
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userDoPelvisPose.getBooleanValue())
            {
               sendPelvisTrajectoryCommand();
               userDoPelvisPose.set(false);
               userStreamPelvisPose.set(false);
               userStreamPelvisOrientation.set(false);
            }
         }
      });

      userStreamPelvisPose.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userStreamPelvisPose.getBooleanValue())
            {
               userDoPelvisPose.set(false);
               userStreamPelvisOrientation.set(false);
            }
         }
      });

      userStreamPelvisOrientation.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userStreamPelvisOrientation.getBooleanValue())
            {
               userDoPelvisPose.set(false);
               userStreamPelvisPose.set(false);
            }
         }
      });

      userDesiredPelvisPose.attachVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userStreamPelvisPose.getBooleanValue())
               sendPelvisTrajectoryCommand();
         }
      });

      userDesiredPelvisPose.getYawPitchRoll().attachVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userStreamPelvisOrientation.getBooleanValue())
               sendPelvisOrientationTrajectoryCommand();
         }
      });

      userDesiredPelvisPoseTrajectoryTime.set(defaultTrajectoryTime);
      parentRegistry.addChild(registry);
   }

   private final PelvisTrajectoryCommand poseCommand = new PelvisTrajectoryCommand();
   private final PelvisOrientationTrajectoryCommand orientationCommand = new PelvisOrientationTrajectoryCommand();
   private final Point3D position = new Point3D();
   private final Quaternion orientation = new Quaternion();
   private final Vector3D zeroVelocity = new Vector3D();

   private void sendPelvisTrajectoryCommand()
   {
      userDesiredPelvisPose.getFramePoseIncludingFrame(framePose);
      framePose.changeFrame(worldFrame);

      double time = userDesiredPelvisPoseTrajectoryTime.getDoubleValue();
      framePose.get(position, orientation);
      poseCommand.getSE3Trajectory().clear(worldFrame);
      poseCommand.getSE3Trajectory().setTrajectoryFrame(worldFrame);
      poseCommand.getSE3Trajectory().addTrajectoryPoint(time, position, orientation, zeroVelocity, zeroVelocity);
      poseCommand.getSE3Trajectory().setExecutionMode(ExecutionMode.OVERRIDE);
      poseCommand.getSE3Trajectory().setCommandId(Packet.VALID_MESSAGE_DEFAULT_ID);
      if (DEBUG)
         System.out.println("Submitting " + poseCommand);
      controllerCommandInputManager.submitCommand(poseCommand);
   }

   private void sendPelvisOrientationTrajectoryCommand()
   {
      userDesiredPelvisPose.getFramePoseIncludingFrame(framePose);
      framePose.changeFrame(worldFrame);

      double time = userDesiredPelvisPoseTrajectoryTime.getDoubleValue();
      orientation.set(framePose.getOrientation());
      orientationCommand.getSO3Trajectory().clear(worldFrame);
      orientationCommand.getSO3Trajectory().setTrajectoryFrame(worldFrame);
      orientationCommand.getSO3Trajectory().addTrajectoryPoint(time, orientation, zeroVelocity);
      orientationCommand.getSO3Trajectory().setExecutionMode(ExecutionMode.OVERRIDE);
      orientationCommand.getSO3Trajectory().setCommandId(Packet.VALID_MESSAGE_DEFAULT_ID);
      if (DEBUG)
         System.out.println("Submitting " + orientationCommand);
      controllerCommandInputManager.submitCommand(orientationCommand);
   }
}
