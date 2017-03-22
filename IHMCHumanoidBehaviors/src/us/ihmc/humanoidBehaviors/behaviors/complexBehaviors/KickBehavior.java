package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootLoadBearingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FootTrajectoryTask;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.taskExecutor.PipeLine;

public class KickBehavior extends AbstractBehavior
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final DoubleYoVariable yoTime;
   private final ReferenceFrame midZupFrame;
   private BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);
   private final FootTrajectoryBehavior footTrajectoryBehavior;

   private FramePoint2d objectToKickPose;

   private final ArrayList<AbstractBehavior> behaviors = new ArrayList<AbstractBehavior>();

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();
   private final DoubleYoVariable trajectoryTime;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   public KickBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super(outgoingCommunicationBridge);
      this.yoTime = yoTime;
      midZupFrame = referenceFrames.getMidFeetZUpFrame();
      trajectoryTime = new DoubleYoVariable("kickTrajectoryTime", registry);
      trajectoryTime.set(0.5);
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();

      footTrajectoryBehavior = new FootTrajectoryBehavior(outgoingCommunicationBridge, yoTime, yoDoubleSupport);
      registry.addChild(footTrajectoryBehavior.getYoVariableRegistry());
   }

   @Override
   public void doControl()
   {
      if (!hasInputBeenSet.getBooleanValue())
      {
         // setObjectToKickPoint(new
         // FramePoint(ReferenceFrame.getWorldFrame(),0,0,0));
         checkInput();
      }
      else
      {
         pipeLine.doControl();
      }
   }

   public void setObjectToKickPoint(FramePoint2d objectToKickPose)
   {
      this.objectToKickPose = objectToKickPose;
   }

   private void checkInput()
   {
      if (objectToKickPose != null)
      {
         hasInputBeenSet.set(true);
         setupPipeline();
      }
   }

   private void setupPipeline()
   {
      final RobotSide kickFoot = RobotSide.LEFT;

      // submitFootPosition(kickFoot.getOppositeSide(), new
      // FramePoint(ankleZUpFrames.get(kickFoot), 0.0,
      // kickFoot.getOppositeSide().negateIfRightSide(0.25), 0.227));
      //
      // submitFootPosition(kickFoot.getOppositeSide(), new
      // FramePoint(ankleZUpFrames.get(kickFoot), 0.0,
      // kickFoot.getOppositeSide().negateIfRightSide(0.35), 0.227));
      //
      //
      //
      // submitFootPosition(kickFoot.getOppositeSide(), new
      // FramePoint(ankleZUpFrames.get(kickFoot), 0.0,
      // kickFoot.getOppositeSide().negateIfRightSide(0.35), 0));

      submitFootPosition(kickFoot, new FramePoint(ankleZUpFrames.get(kickFoot.getOppositeSide()), 0.0, kickFoot.negateIfRightSide(0.25), 0.227));

      submitFootPosition(kickFoot, new FramePoint(ankleZUpFrames.get(kickFoot.getOppositeSide()), -0.2, kickFoot.negateIfRightSide(0.15), 0.127));

      submitFootPosition(kickFoot, new FramePoint(ankleZUpFrames.get(kickFoot.getOppositeSide()), 0.3, kickFoot.negateIfRightSide(0.15), 0.05));

      // submitFootPosition(kickFoot, new
      // FramePoint(objectToKickPose.getReferenceFrame(),
      // objectToKickPose.getX(), objectToKickPose.getY(), 0.127));

      submitFootPosition(kickFoot, new FramePoint(ankleZUpFrames.get(kickFoot.getOppositeSide()), 0.0, kickFoot.negateIfRightSide(0.25), 0.127));

      submitFootPosition(kickFoot, new FramePoint(ankleZUpFrames.get(kickFoot.getOppositeSide()), 0.0, kickFoot.negateIfRightSide(0.25), 0));

      final FootLoadBearingBehavior footStateBehavior = new FootLoadBearingBehavior(communicationBridge);
      pipeLine.submitSingleTaskStage(new BehaviorAction(footStateBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            FootLoadBearingMessage message = new FootLoadBearingMessage(kickFoot, LoadBearingRequest.LOAD);
            footStateBehavior.setInput(message);

         }
      });
   }

   private void submitFootPosition(RobotSide robotSide, FramePoint desiredFootPosition)
   {
      FrameOrientation desiredFootOrientation = new FrameOrientation(desiredFootPosition.getReferenceFrame());
      FramePose desiredFootPose = new FramePose(desiredFootPosition, desiredFootOrientation);
      submitFootPose(robotSide, desiredFootPose);
   }

   private void submitFootPose(RobotSide robotSide, FramePose desiredFootPose)
   {
      desiredFootPose.changeFrame(worldFrame);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      desiredFootPose.getPose(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryTask task = new FootTrajectoryTask(robotSide, desiredFootPosition, desiredFootOrientation, footTrajectoryBehavior,
            trajectoryTime.getDoubleValue());
      pipeLine.submitSingleTaskStage(task);
   }

   @Override
   public void onBehaviorEntered()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorEntered();
      }
   }

   @Override
   public void onBehaviorExited()
   {
      hasInputBeenSet.set(false);
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorExited();
      }
   }

   @Override
   public void onBehaviorAborted()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorAborted();
      }
   }

   @Override
   public void onBehaviorPaused()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorPaused();
      }
   }

   @Override
   public boolean isDone()
   {
      return hasInputBeenSet() && pipeLine.isDone();
   }

   @Override
   public void onBehaviorResumed()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorResumed();
      }
   }



   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
