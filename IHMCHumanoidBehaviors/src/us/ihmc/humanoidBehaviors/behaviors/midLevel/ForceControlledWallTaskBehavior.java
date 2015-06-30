/**
 * 
 * next steps:
 *    send handposepacket to controller
 *    wait for until done() (from controllerstate? or from trajectory?)
 *    send new handposepacket
 *    implement controller in taskspaceToJointspaceHandForcefeedbackControlState
 * 
 * 
 */

package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.communication.packets.manipulation.HandPoseStatus.Status;
import us.ihmc.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.communication.packets.manipulation.HandRotateAboutAxisPacket.DataType;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePose;


/**
 * 
 * @author Tobias Meier
 * 
 *
 */



public class ForceControlledWallTaskBehavior extends BehaviorInterface
{
   private final ConcurrentListeningQueue<HandPoseStatus> handStatusListeningQueue = new ConcurrentListeningQueue<HandPoseStatus>();
   protected Status status;
   private BooleanYoVariable isDone;
   
   private final FullRobotModel fullrobotModel;
   private final DoubleYoVariable yoTime;
   private final YoGraphicsListRegistry visualizerYoGraphicsRegistry;
   private final YoGraphicCoordinateSystem yoTrajectoryControlFrame;
   private final YoFramePose yoHandPose;

   // Reference Frames
   private final ReferenceFrame worldFrame;
   private final ReferenceFrame handControlFrame;

   // Handpose commands
   private HandPosePacket straightLineControlCmd;
   private HandRotateAboutAxisPacket circleControlCmd;

   // Circle commands
   private Point3d rotationAxisOriginInWorld;
   private Vector3d rotationAxisInWorld;
   private Vector3d startToCenter = new Vector3d(0.0, 0.0, 0.20);
   
   private FramePose startPose = new FramePose();
   private Point3d nextCoordinate = new Point3d();
   private Quat4d nextOrientation = new Quat4d();
   
   private Vector3d tempVector = new Vector3d();
   private FramePoint tempFramePoint = new FramePoint();
   private final static double EPSILON = 5.0e-3; 
   

   private enum BehaviorStates {SET_STARTPOSITION_IN_SIM, WAIT_FOR_STARTPOSITION, WAIT_FOR_REACHING_GOAL, INITIALIZE, SEND_COMMAND_TO_CONTROLLER, DONE}
   private BehaviorStates behaviorState;
   private final RobotSide robotSide;



   public ForceControlledWallTaskBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, ReferenceFrames referenceFrames, FullRobotModel fullRobotModel, DoubleYoVariable yoTime, YoGraphicsListRegistry yoGraphicsRegistry)
   {
      super(outgoingCommunicationBridge);
      this.fullrobotModel = fullRobotModel;
      this.yoTime = yoTime;
      this.visualizerYoGraphicsRegistry = yoGraphicsRegistry;
      this.robotSide = RobotSide.RIGHT;
      this.attachControllerListeningQueue(handStatusListeningQueue, HandPoseStatus.class);

      isDone = new BooleanYoVariable("isDone", registry);
      // Reference Frames:
      worldFrame = ReferenceFrames.getWorldFrame();
      handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      
      
      yoHandPose = new YoFramePose("yoHandPose", worldFrame, registry);
      yoTrajectoryControlFrame = new YoGraphicCoordinateSystem("yoTrajectoryControlFrame", yoHandPose, 0.2);
      visualizerYoGraphicsRegistry.registerYoGraphic("yoTrajectoryControlFrameViz", yoTrajectoryControlFrame);
      
      straightLineControlCmd = new HandPosePacket(robotSide, Frame.WORLD, null, null, 1.0);
      
      
      circleControlCmd = new HandRotateAboutAxisPacket();
      rotationAxisInWorld = new Vector3d();
      rotationAxisOriginInWorld = new Point3d();
      
   // Starting point for simulation.
      startPose.setToZero(worldFrame);
      startPose.setPosition(0.4, -0.0, 0.75);
      Matrix3d RotationMatrix = new Matrix3d();
      if(robotSide == RobotSide.RIGHT)
      {
         RotationMatrix.rotZ(Math.PI / 2.0);
         startPose.setOrientation(RotationMatrix);
         
      }
      else if(robotSide == RobotSide.LEFT)
      {
         RotationMatrix.rotZ(-Math.PI / 2.0);
         startPose.setOrientation(RotationMatrix);
      }
      else
      {
         PrintTools.error(this, "robotSide not defined.");
      }
      startPose.getPosition(nextCoordinate);
      startPose.getOrientation(nextOrientation);
      
      
      straightLineControlCmd.position = nextCoordinate;
      straightLineControlCmd.orientation = nextOrientation;
      straightLineControlCmd.trajectoryTime = 5.0;

      behaviorState = BehaviorStates.SET_STARTPOSITION_IN_SIM;



   }

   @Override
   public void doControl()
   {
      if (handStatusListeningQueue.isNewPacketAvailable())
      {
         consumeHandPoseStatus(handStatusListeningQueue.getNewestPacket());
      }
      
      switch(behaviorState)
      {
      case SET_STARTPOSITION_IN_SIM:
         sendPacketToController(straightLineControlCmd);
         PrintTools.debug(this, "Sent startposition to controller.");
         behaviorState = BehaviorStates.WAIT_FOR_STARTPOSITION;
         break;
      
      case WAIT_FOR_STARTPOSITION:
         tempFramePoint.setToZero(handControlFrame);
         tempFramePoint.changeFrame(worldFrame);
         tempVector.set(tempFramePoint.getPoint().x, tempFramePoint.getPoint().y, tempFramePoint.getPoint().z);
         tempVector.sub(nextCoordinate);
         
         if(tempVector.length() < EPSILON)
         {
            behaviorState = BehaviorStates.INITIALIZE;
         }
         break;
   
      case INITIALIZE:

         startPose.getPosition(rotationAxisOriginInWorld);
         rotationAxisOriginInWorld.add(startToCenter);
         rotationAxisInWorld.set(1.0,  0.0, 0.0);
         
         initializeForceControlCircle(rotationAxisOriginInWorld, rotationAxisInWorld, -5.0, -3.0);
         
         
         
         behaviorState = BehaviorStates.SEND_COMMAND_TO_CONTROLLER;
         break;
         

      case SEND_COMMAND_TO_CONTROLLER:

         sendPacketToController(circleControlCmd);
         
         behaviorState = BehaviorStates.WAIT_FOR_REACHING_GOAL;

         break;
      case WAIT_FOR_REACHING_GOAL:
         if (status == Status.COMPLETED)
         {
            behaviorState = BehaviorStates.DONE;
         }
         
         break;

      case DONE:
         isDone.set(true);
         break;

      default:
         PrintTools.error(this, "No valid behavior state.");

         break;

      }

   }

   private void initializeForceControlCircle(Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld, double tangentialForce, double normalForce)
   {
//      straightLineControlCmd.dataType = straightLineControlCmd.dataType.HAND_POSE_FORCECONTROL;
//      straightLineControlCmd.desiredTangentialForce = -1.0;
//      straightLineControlCmd.forceConstraint = new Vector3d(0.0, 0.0, 0.0);
      
//      circleControlCmd.setForceControl(-5.0, -3.0);
      
      circleControlCmd.graspOffsetFromControlFrame = 0.0;
      circleControlCmd.robotSide = robotSide;
      circleControlCmd.trajectoryTime = 10.0;
      circleControlCmd.controlHandAngleAboutAxis = false;
      circleControlCmd.rotationAxisInWorld = rotationAxisInWorld;
      circleControlCmd.rotationAxisOriginInWorld = rotationAxisOriginInWorld;
      circleControlCmd.rotationRightHandRule = 2.0*Math.PI;
      
      Vector3d normalForceVector = new Vector3d(rotationAxisInWorld);
      normalForceVector.scale(normalForce);
      
//      circleControlCmd.setForceControlParameters(tangentialForce, normalForceVector);
      
   }
   
   private void consumeHandPoseStatus(HandPoseStatus handPoseStatus)
   {
         RobotSide statusRobotSide = handPoseStatus.getRobotSide();
         
         if (statusRobotSide == robotSide)
         {
            status = handPoseStatus.getStatus();
         }
   }


   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      // TODO Auto-generated method stub

   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void stop()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void enableActions()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void pause()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void resume()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isDone()
   {
       return isDone.getBooleanValue();
   }

   @Override
   public void finalize()
   {
      status = null;
      isDone.set(false);
   }

   @Override
   public void initialize()
   {
      status = null;

   }

   @Override
   public boolean hasInputBeenSet()
   {
      // TODO Auto-generated method stub
      return false;
   }

}

