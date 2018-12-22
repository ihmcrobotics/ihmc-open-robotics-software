package us.ihmc.quadrupedUI.controllers;

import controller_msgs.msg.dds.QuadrupedBodyTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedUI.QuadrupedUIMessagerAPI;
import us.ihmc.tools.inputDevices.joystick.Joystick;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class BodyPoseController
{
   static final double DT = 0.05;

   private static final boolean doBodyHeightControl = false;
   private static final boolean doXYTranslation = false;
   private static final boolean doBodyOrientationControl = true;
   private static final double bodyOrientationMoveTime = 0.35;

   private final AtomicBoolean disableBodyControl = new AtomicBoolean(false);
   private final ScheduledExecutorService executorService = Executors
         .newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicReference<QuadrupedReferenceFrames> referenceFramesReference;

   private ScheduledFuture<?> serviceFuture;
   private BodyControlXBoxAdapter xBoxAdapter;

   private final Messager messager;

   public BodyPoseController(Joystick joystick, JavaFXMessager messager, double nominalHeight)
   {
      this.messager = messager;

      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableBodyControlTopic, this::conditionallyEnableBodyControl);

      referenceFramesReference = messager.createInput(QuadrupedUIMessagerAPI.ReferenceFramesTopic);
      xBoxAdapter = new BodyControlXBoxAdapter(joystick, nominalHeight);
   }

   private void conditionallyEnableBodyControl(boolean enable)
   {
      if (!enable)
      {
         disableBodyControl.set(true);
      }
      else
      {
         serviceFuture = executorService.scheduleAtFixedRate(() -> {
            try
            {
               update();
            }
            catch (Exception e)
            {
            }
         }, 0, (long) (DT * 1000), TimeUnit.MILLISECONDS);
      }
   }

   private void update()
   {
      if (disableBodyControl.getAndSet(false))
      {
         serviceFuture.cancel(true);
      }

      // joystick commands
      if (xBoxAdapter == null)
      {
         serviceFuture.cancel(true);
         return;
      }

      xBoxAdapter.update();
      handleBodyHeightCommand(xBoxAdapter.getCommandedBodyHeight());
      handleBodyTrajectoryCommand(xBoxAdapter.getCommandedBodyYaw(), xBoxAdapter.getCommandedBodyPitch(), xBoxAdapter.getCommandedBodyRoll(),
                                  xBoxAdapter.getCommandedBodyTranslationX(), xBoxAdapter.getCommandedBodyTranslationY());
   }

   private final FramePoint3D tempPoint = new FramePoint3D();

   private void handleBodyHeightCommand(double commandedBodyHeight)
   {
      if (Double.isNaN(commandedBodyHeight) || !doBodyHeightControl)
         return;

      tempPoint.setIncludingFrame(referenceFramesReference.get().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds(), 0.0, 0.0, commandedBodyHeight);
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());

      messager.submitMessage(QuadrupedUIMessagerAPI.DesiredBodyHeightTopic, tempPoint.getZ());
   }

   private void handleBodyTrajectoryCommand(double commandedBodyYaw, double commandedBodyPitch, double commandedBodyRoll, double commandedTranslationX,
                                            double commandedTranslationY)
   {
      if (Double.isNaN(commandedBodyYaw) || Double.isNaN(commandedBodyPitch) || Double.isNaN(commandedBodyRoll) || Double.isNaN(commandedTranslationX) || Double
            .isNaN(commandedTranslationY))
         return;

      QuadrupedBodyTrajectoryMessage bodyTrajectoryMessage = new QuadrupedBodyTrajectoryMessage();

      bodyTrajectoryMessage.setIsExpressedInAbsoluteTime(false);
      bodyTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().clear();
      SE3TrajectoryPointMessage trajectoryPointMessage = bodyTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add();
      trajectoryPointMessage.getOrientation().setYawPitchRoll(commandedBodyYaw, commandedBodyPitch, commandedBodyRoll);
      trajectoryPointMessage.getPosition().set(commandedTranslationX, commandedTranslationY, 0.0);

      bodyTrajectoryMessage.getSe3Trajectory().getLinearSelectionMatrix().setXSelected(doXYTranslation);
      bodyTrajectoryMessage.getSe3Trajectory().getLinearSelectionMatrix().setYSelected(doXYTranslation);
      bodyTrajectoryMessage.getSe3Trajectory().getLinearSelectionMatrix().setZSelected(false);
      bodyTrajectoryMessage.getSe3Trajectory().getAngularSelectionMatrix().setXSelected(doBodyOrientationControl);
      bodyTrajectoryMessage.getSe3Trajectory().getAngularSelectionMatrix().setYSelected(doBodyOrientationControl);
      bodyTrajectoryMessage.getSe3Trajectory().getAngularSelectionMatrix().setZSelected(doBodyOrientationControl);
      trajectoryPointMessage.setTime(bodyOrientationMoveTime);

      messager.submitMessage(QuadrupedUIMessagerAPI.BodyTrajectoryMessageTopic, bodyTrajectoryMessage);
   }
}
