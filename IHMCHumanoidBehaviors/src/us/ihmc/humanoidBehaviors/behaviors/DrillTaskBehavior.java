package us.ihmc.humanoidBehaviors.behaviors;

import javax.vecmath.Vector3d;

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
import us.ihmc.humanoidBehaviors.taskExecutor.WholeBodyInverseKinematicTask;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class DrillTaskBehavior extends BehaviorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final DoubleYoVariable yoTime;
   private final ConcurrentListeningQueue<DrillTaskPacket> drillTaskBehaviorInputPacketListener;
   private final ReferenceFrame midZupFrame;
   private BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);
   
   private RigidBodyTransform drillTransform;
   private final FramePose2d pickUpPose2dInWorld;
   private final FramePose readyToGrabInWorld;
   private final FramePose grabInWorld;
   
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior;
   private final FingerStateBehavior fingerStateBehavior;
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();
   
   private final RobotSide grabSide = RobotSide.RIGHT;
   private final double getReadyToGrabTime = 1.5;
   private final double grabTime = 1.0;
   
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
      pickUpPose2dInWorld = new FramePose2d(worldFrame);
      readyToGrabInWorld = new FramePose(worldFrame);
      grabInWorld = new FramePose(worldFrame);
      
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
         drillTransform = drillTaskBehaviorInputPacketListener.getNewestPacket().getDrillTransform();
         drillTaskBehaviorInputPacketListener.clear();
         
         if(drillTransform != null)
         {
            hasInputBeenSet.set(true);
            
            pipeLine.requestNewStage();
            
            pickUpPose2dInWorld.setPose(drillTransform);
            pickUpPose2dInWorld.changeFrame(midZupFrame);
            pickUpPose2dInWorld.setX(pickUpPose2dInWorld.getX() - 0.85);
            pickUpPose2dInWorld.setY(pickUpPose2dInWorld.getY() - 0.0);
            pickUpPose2dInWorld.changeFrame(worldFrame);
            
            pipeLine.submitSingleTaskStage(new WalkToLocationTask(pickUpPose2dInWorld, walkToLocationBehavior,
                  0.0, 0.3, yoTime));
            
            pipeLine.requestNewStage();
            
            Vector3d drillPosition = new Vector3d();
            drillTransform.getTranslation(drillPosition);
            readyToGrabInWorld.setPosition(drillPosition);
            readyToGrabInWorld.changeFrame(midZupFrame);
            readyToGrabInWorld.setX(readyToGrabInWorld.getX() - 0.4);
            readyToGrabInWorld.setZ(readyToGrabInWorld.getZ() + 0.15);
            readyToGrabInWorld.changeFrame(worldFrame);
            
            pipeLine.submitSingleTaskStage(new WholeBodyInverseKinematicTask(grabSide, yoTime, wholeBodyIKBehavior,
                  readyToGrabInWorld, getReadyToGrabTime, 0, ControlledDoF.DOF_3P2R));
            
            pipeLine.requestNewStage();
            
            grabInWorld.setPose(readyToGrabInWorld);
            grabInWorld.changeFrame(midZupFrame);
            grabInWorld.setX(grabInWorld.getX() + 0.25);
            grabInWorld.changeFrame(worldFrame);
            
            pipeLine.submitSingleTaskStage(new WholeBodyInverseKinematicTask(grabSide, yoTime, wholeBodyIKBehavior,
                  grabInWorld, grabTime, 0, ControlledDoF.DOF_3P2R));
            
            pipeLine.requestNewStage();
            
            pipeLine.submitSingleTaskStage(new FingerStateTask(grabSide, FingerState.CLOSE, fingerStateBehavior, yoTime));
            
            pipeLine.requestNewStage();
         }
      }
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
