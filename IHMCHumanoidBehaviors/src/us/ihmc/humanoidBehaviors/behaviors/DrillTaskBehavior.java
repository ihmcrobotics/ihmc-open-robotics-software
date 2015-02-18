package us.ihmc.humanoidBehaviors.behaviors;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.DrillTaskPacket;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class DrillTaskBehavior extends BehaviorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final DoubleYoVariable yoTime;
   private final ConcurrentListeningQueue<DrillTaskPacket> drillTaskBehaviorInputPacketListener;
   private final ReferenceFrame midZupFrame;
   private BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);
   
   private FramePose drillPose;
   
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final FramePose2d pickUpPoseToWalkTo;
   
   private final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior;
   private final FingerStateBehavior fingerStateBehavior;
   
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();
   
   private final RobotSide grabSide = RobotSide.RIGHT;
   private final double grabDistance = 0.9;
   
   public DrillTaskBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime,
         SDFFullRobotModel fullRobotModel, ReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters,
         WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);
      drillTaskBehaviorInputPacketListener = new ConcurrentListeningQueue<DrillTaskPacket>();
      this.yoTime = yoTime;
      
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      registry.addChild(walkToLocationBehavior.getYoVariableRegistry());
      walkToLocationBehavior.setDistanceThreshold(0.1);
      walkToLocationBehavior.setYawAngleThreshold(Math.toRadians(5.0));
      
      wholeBodyIKBehavior = new WholeBodyInverseKinematicBehavior(outgoingCommunicationBridge, wholeBodyControllerParameters, fullRobotModel, yoTime);
      registry.addChild(wholeBodyIKBehavior.getYoVariableRegistry());
      wholeBodyIKBehavior.setPositionAndOrientationErrorTolerance(0.05, 0.3);
      
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      registry.addChild(fingerStateBehavior.getYoVariableRegistry());
      
      midZupFrame = referenceFrames.getMidFeetZUpFrame();
      pickUpPoseToWalkTo = new FramePose2d(worldFrame);
      
      super.attachNetworkProcessorListeningQueue(drillTaskBehaviorInputPacketListener, DrillTaskPacket.class);
   }

   @Override
   public void doControl()
   {
      if (!hasInputBeenSet.getBooleanValue())
      {
         checkInput();
      }
      else
      {
         pipeLine.doControl();
      }
   }
   
   private void checkInput()
   {
      if (drillTaskBehaviorInputPacketListener.isNewPacketAvailable())
      {
         RigidBodyTransform drillTransform = drillTaskBehaviorInputPacketListener.getNewestPacket().getDrillTransform();
         drillTaskBehaviorInputPacketListener.clear();
         
         if(drillTransform != null)
         {
            drillPose = new FramePose(worldFrame, drillTransform);
            hasInputBeenSet.set(true);
            setupPipeline();
         }
      }
   }
   
   private void setupPipeline()
   {
      pipeLine.requestNewStage();
      
      // stage 1: position robot on front of drill
      FramePose2d drillPose2d = new FramePose2d(worldFrame);
      drillPose.getPose2dIncludingFrame(drillPose2d);
      FramePoint2d drillPosition = new FramePoint2d();
      drillPose2d.getPosition(drillPosition);
      FramePoint2d robotPosition = new FramePoint2d(midZupFrame, 0.0, 0.0);
      robotPosition.changeFrame(worldFrame);
      FrameVector2d walkingDirection = new FrameVector2d(worldFrame);
      walkingDirection.set(drillPosition);
      walkingDirection.sub(robotPosition);
      walkingDirection.normalize();
      double yaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
      double x = drillPosition.getX() - walkingDirection.getX() * grabDistance;
      double y = drillPosition.getY() - walkingDirection.getY() * grabDistance;
      pickUpPoseToWalkTo.setPoseIncludingFrame(worldFrame, x, y, yaw);
      
      pipeLine.submitSingleTaskStage(new WalkToLocationTask(pickUpPoseToWalkTo, walkToLocationBehavior, 0.0, 0.3, yoTime));
      pipeLine.requestNewStage();
      
      // stage 2: bring hand into position to grab and open it
//      readyToGrabPose.setPose(drillPose);
//      readyToGrabPose.setOrientation(yaw + Math.PI / 4, 0.0, 0.0);
//      readyToGrabPose.changeFrame(midZupFrame);
//      readyToGrabPose.setX(readyToGrabPose.getX() - 0.4);
//      readyToGrabPose.setZ(readyToGrabPose.getZ() + 0.15);
//      readyToGrabPose.changeFrame(worldFrame);
//      
//      pipeLine.submitTaskForPallelPipesStage(wholeBodyIKBehavior, new WholeBodyInverseKinematicTask(grabSide, yoTime, wholeBodyIKBehavior, readyToGrabPose, getReadyToGrabTime, 0, ControlledDoF.DOF_3P2R));
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(grabSide, FingerState.OPEN, fingerStateBehavior, yoTime));
      pipeLine.requestNewStage();
      
      // stage 3: move hand closer to drill
//      grabPose.setPose(readyToGrabPose);
//      grabPose.changeFrame(midZupFrame);
//      grabPose.setX(grabPose.getX() + 0.25);
//      grabPose.changeFrame(worldFrame);
//      
//      pipeLine.submitSingleTaskStage(new WholeBodyInverseKinematicTask(grabSide, yoTime, wholeBodyIKBehavior,
//            grabPose, grabTime, 0, ControlledDoF.DOF_3P2R));
//      pipeLine.requestNewStage();
      
      // stage 4: close hand
      pipeLine.submitSingleTaskStage(new FingerStateTask(grabSide, FingerState.CLOSE, fingerStateBehavior, yoTime));
      pipeLine.requestNewStage();
   }

   @Override
   public void initialize()
   {
      defaultInitialize();
      walkToLocationBehavior.initialize();
      wholeBodyIKBehavior.initialize();
   }

   @Override
   public void finalize()
   {
      defaultFinalize();
      walkToLocationBehavior.finalize();
      wholeBodyIKBehavior.finalize();
   }

   @Override
   public void stop()
   {
      defaultStop();
      walkToLocationBehavior.stop();
      wholeBodyIKBehavior.stop();
   }

   @Override
   public void pause()
   {
      defaultPause();
      walkToLocationBehavior.pause();
      wholeBodyIKBehavior.pause();
   }

   @Override
   public boolean isDone()
   {
      return defaultIsDone() && pipeLine.isDone();
   }

   @Override
   public void enableActions()
   {

   }

   @Override
   public void resume()
   {
      defaultResume();
      walkToLocationBehavior.resume();
      wholeBodyIKBehavior.resume();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      walkToLocationBehavior.passReceivedNetworkProcessorObjectToChildBehaviors(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      walkToLocationBehavior.passReceivedControllerObjectToChildBehaviors(object);
   }

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
