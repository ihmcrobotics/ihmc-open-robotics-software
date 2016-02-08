package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.HandDesiredConfigurationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ButtonData;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorButtonPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.taskExecutor.PipeLine;

public class PushButtonBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;
   
   
   

   private static final boolean OVERRIDE = true;
   private static final boolean FORCE_CONTROL = true;
   private static final double TRAJECTORY_TIME = 1.0;


   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();
   private final ConcurrentListeningQueue<HumanoidBehaviorButtonPacket> inputListeningQueue = new ConcurrentListeningQueue<HumanoidBehaviorButtonPacket>();
   private final SideDependentList<WristForceSensorFilteredUpdatable> wristSensors;

   private final DoubleYoVariable yoTime;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final HandPoseBehavior handPoseBehavior;
   private final HandDesiredConfigurationBehavior fingerStateBehavior;
   private HandDesiredConfigurationTask closeHandTask;


   private ArrayList<Vector3d> pushDirections = new ArrayList<Vector3d>();
   private ArrayList<Point3d> pushLocations = new ArrayList<Point3d>();

   private BooleanYoVariable isDone;
   private BooleanYoVariable haveInputsBeenSet;
   private boolean behaviorDone = false;
   private RobotSide robotSide;
   private int buttonNumber;

   private FramePose newHandPose = new FramePose(worldFrame);
   private FramePose initialHandPose = new FramePose(worldFrame);
   private Matrix3d poseZRotation =  new Matrix3d();
   private Matrix3d poseYRotation =  new Matrix3d();
   private Matrix3d poseRotation =  new Matrix3d();

   private Point3d newHandPosition = new Point3d();
   private Point3d initialHandPosition = new Point3d();
   private Vector3d pushVector = new Vector3d();
   private final Vector3d wristOffset = new Vector3d();

   private static final double HANDPOSE_ERROR = 0.001;
   private static final double RETRACT_HANDPOSE_ERROR = 0.01;

   private enum ControllerStates {REQUEST_INPUTS, INITIALIZE_HANDPOSE, WAIT_FOR_HANDPOSE, DO_FORCECONTROL, WAIT_FOR_RETRACTION, DO_POSITIONCONTROL, DONE};
   ControllerStates controllerState;

   // Force controller specifics
   private AlphaFilteredYoVariable forceMeasurement;
   private final double ALPHA = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0, IHMCHumanoidBehaviorManager.BEHAVIOR_YO_VARIABLE_SERVER_DT);

   private static final double DESIRED_END_FORCE = 5.0;  // minimum force to switch button: 2.375, but main point is to make sure the robot does not tip over.
   private static final double END_FORCE_ERROR = 0.1;
   private double deltaF = DESIRED_END_FORCE;
   private double P = 1/200000.0;
   private Vector3d distanceVector = new Vector3d();
   private Quat4d orientation = new Quat4d();
   private FramePose oldHandPose;
   private HandPosePacket controlCmds;
   private int controllerCounter;


   public PushButtonBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge,
         HumanoidReferenceFrames referenceFrames, DoubleYoVariable yoTime, SideDependentList<WristForceSensorFilteredUpdatable> wristSensors)
   {
      super(outgoingCommunicationBridge);

      robotSide = RobotSide.RIGHT;

      this.wristSensors = new SideDependentList<WristForceSensorFilteredUpdatable>(wristSensors);
      this.yoTime = yoTime;
      forceMeasurement =  new AlphaFilteredYoVariable("PushForce", registry, ALPHA);

      buttonNumber = 0;

      controllerState = ControllerStates.REQUEST_INPUTS;

      if(FORCE_CONTROL)
      {
         controlCmds = new HandPosePacket(robotSide, Frame.WORLD, null, null, 0.1);
         oldHandPose = new FramePose(initialHandPose);
      }

      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      fingerStateBehavior = new HandDesiredConfigurationBehavior(outgoingCommunicationBridge, yoTime);

      closeHandTask = new HandDesiredConfigurationTask(robotSide,HandConfiguration.CLOSE, fingerStateBehavior, yoTime);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
      haveInputsBeenSet.set(false);

      isDone = new BooleanYoVariable("isDone", registry);

      attachNetworkProcessorListeningQueue(inputListeningQueue, HumanoidBehaviorButtonPacket.class);

      if(DEBUG)
      {
         System.out.println("ControllerState: " + controllerState);
      }
   }

   @Override
   public void doControl()
   {
      switch (controllerState) {
      case REQUEST_INPUTS:
         checkForNewInputs();
         break;

      case INITIALIZE_HANDPOSE:
         setInitialHandPose();
         if(DEBUG)
         {
            System.out.println("Memory free [MB]/ Memory total [MB]: " + Runtime.getRuntime().freeMemory() / 1024 / 1024 + " / " + Runtime.getRuntime().totalMemory() / 1024 / 1024);
         }
         break;

      case WAIT_FOR_HANDPOSE:
         distanceVector.set(wristSensors.get(robotSide).getWristPositionInWorld().getPointCopy());
         distanceVector.add(wristOffset);
         distanceVector.sub(newHandPosition);
         if(DEBUG)
         {
            System.out.println("Distance to required pose:" + distanceVector.length());
         }

         if(distanceVector.length() < HANDPOSE_ERROR){
            controllerState = ControllerStates.DO_FORCECONTROL;
            controllerCounter = 0;
            if(DEBUG){
               System.out.println("ControllerState: " + controllerState);
            }
         }
         break;

      case WAIT_FOR_RETRACTION:
         distanceVector.set(wristSensors.get(robotSide).getWristPositionInWorld().getPointCopy());
         distanceVector.add(wristOffset);
         distanceVector.sub(initialHandPosition);

         if(distanceVector.length() < RETRACT_HANDPOSE_ERROR){
            controllerState = ControllerStates.INITIALIZE_HANDPOSE;
            if(DEBUG){
               System.out.println("ControllerState: " + controllerState);
            }
         }
         break;

      case DO_FORCECONTROL:
         forceController();
         break;

      case DO_POSITIONCONTROL:
         break;

      case DONE:
         submitSingleTaskStageHandPose(initialHandPose,robotSide);
         behaviorDone = true;

      default:
         break;
      }

      pipeLine.doControl();
   }

   private void checkForNewInputs()
   {
      HumanoidBehaviorButtonPacket newestPacket = inputListeningQueue.getNewestPacket();

      if (OVERRIDE)
      {
         // Button in GUI does not have to be placed
         pushLocations.add(new Point3d(0.8, -0.3, 1.1));
         pushDirections.add(new Vector3d(1.0, 0.0, 0.0));
         controllerState = ControllerStates.INITIALIZE_HANDPOSE;
      }
      else if(newestPacket != null)
      {

         setInputs(newestPacket);
         if(DEBUG)
         {
            System.out.println("ControllerState: " + controllerState + "; Check for new Inputs sucessful");
         }

      }

      if(DEBUG)
      {
         System.out.println("ControllerState: " + controllerState + "; Check for new Inputs not sucessful");
      }

   }

   public void setInitialHandPose()
   {
      if (pushLocations.size() == buttonNumber)
      {
         controllerState = ControllerStates.DONE;

         if(DEBUG)
         {
            System.out.println("No more buttons.");
            System.out.println("ControllerState: " + controllerState);
         }
         return;
      }

      if(DEBUG)
      {
         System.out.println("ButtonNumber: " + buttonNumber);
         System.out.println("ControllerState: " + controllerState);
      }

      pushVector.set(pushDirections.get(buttonNumber));
      pushVector.normalize();
      newHandPosition =  new Point3d(pushLocations.get(buttonNumber));

      // Place hand 0.05 away from assumed button position to avoid early collision.
      pushVector.scale(0.05);
      newHandPosition.sub(pushVector);
      pushVector.normalize();
      if(DEBUG)
      {
         System.out.println("Initial Hand Pose for Button " + buttonNumber +" set: " + newHandPosition + "; Push direction: " + this.pushVector);
      }

      double alpha = Math.atan(pushVector.getY() / pushVector.getX());
      double beta = - Math.atan(pushVector.getZ() / Math.sqrt(Math.pow(pushVector.getX(), 2.0) + Math.pow(pushVector.getY(),2.0)));
      poseZRotation.rotZ(alpha);
      poseYRotation.rotY(beta);
      poseRotation.mul(poseZRotation, poseYRotation);

      // Transform the wrist offset
      wristOffset.set(0.26, 0.0, 0.0);
      poseRotation.transform(wristOffset);

      initialHandPose.setOrientation(poseRotation);
      initialHandPose.setPosition(newHandPosition);

      pipeLine.clearAll();

      // Form fist:
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, closeHandTask);

      // Set initial pose:
      submitSingleTaskStageHandPose(initialHandPose,robotSide);

      newHandPose = new FramePose(initialHandPose);

      if (FORCE_CONTROL)
      {
         controllerState = ControllerStates.WAIT_FOR_HANDPOSE;

         oldHandPose = new FramePose(initialHandPose);
         newHandPose = new FramePose(initialHandPose);
      }
      else 
      {
         controllerState = ControllerStates.DO_POSITIONCONTROL;
         Point3d handPoseEndPosition = new Point3d(newHandPosition);
         pushVector.scale(0.1);
         handPoseEndPosition.add(pushVector);
         pushVector.normalize();
         newHandPose.setPosition(handPoseEndPosition);

         submitSingleTaskStageHandPose(newHandPose, robotSide);
      }

      if(DEBUG)
      {
         System.out.println("ControllerState: " + controllerState);
      }
   }


   private void forceController()
   {    
      // Take only the force component in push direction
      forceMeasurement.update(wristSensors.get(robotSide).getWristForceMassCompensatedInWorld().dot(pushVector));

      deltaF = DESIRED_END_FORCE - forceMeasurement.getDoubleValue();
      if(DEBUG)
      {
         System.out.println("Filtered deltaF: " + deltaF);
      }

      pushVector.scale(deltaF * P);
      oldHandPose.getPosition(newHandPosition);
      newHandPosition.add(pushVector);

      newHandPose.setPosition(newHandPosition);
      newHandPose.getOrientation(orientation);

      pushVector.normalize();
      controlCmds.position = newHandPosition;
      controlCmds.orientation = orientation;
      oldHandPose.setPose(newHandPose);

      sendPacketToController(controlCmds);

      if(deltaF < END_FORCE_ERROR && controllerCounter > 50)
      {
         buttonNumber++;

         submitSingleTaskStageHandPose(initialHandPose,robotSide);
         initialHandPose.getPosition(initialHandPosition);
         controllerState = ControllerStates.WAIT_FOR_RETRACTION;

         if(DEBUG)
         {
            System.out.println("Desired Force reached: DeltaF = " +  deltaF);
            System.out.println("ControllerState: " + controllerState);
         }
      }
      controllerCounter++;
   }

   private void submitSingleTaskStageHandPose(FramePose handPose, RobotSide side)
   {
      handPose.changeFrame(worldFrame);
      pipeLine.submitSingleTaskStage(new HandPoseTask(side, TRAJECTORY_TIME, handPose, Frame.WORLD, handPoseBehavior, yoTime));
   }

   public void setRobotSide(RobotSide side)
   {
      robotSide = side;
   }

   public void setInputs(HumanoidBehaviorButtonPacket newestButtonPacket)
   {
      for(ButtonData buttonData : newestButtonPacket.buttonDataList)
      {
         this.pushLocations.add(buttonData.getPushPosition());
         this.pushDirections.add(buttonData.getPushDirection());
         controllerState = ControllerStates.INITIALIZE_HANDPOSE;
      }

      if(DEBUG)
      {
         System.out.println("DEBUG: Inputs have been Set!!!!");
      }
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromNetworkProcessor(object);
      fingerStateBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromController(object);
      fingerStateBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      handPoseBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      handPoseBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      handPoseBehavior.pause();
   }

   @Override
   public void resume()
   {
      handPoseBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      return behaviorDone;
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      haveInputsBeenSet.set(false);
      isDone.set(false);
   }

   @Override
   public void initialize()
   {
      pipeLine.clearAll();
      haveInputsBeenSet.set(false);
      isDone.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }

   public boolean isOverrideActive()
   {
      return OVERRIDE;
   }
}
