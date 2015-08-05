package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.behaviors.WallTaskBehaviorData;
import us.ihmc.communication.packets.behaviors.WallTaskBehaviorData.Commands;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.communication.packets.manipulation.HandPoseStatus.Status;
import us.ihmc.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.humanoidRobot.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePose;


/**
 * @author Tobias Meier
 */

public class ForceControlledWallTaskBehavior extends BehaviorInterface
{
	private final ConcurrentListeningQueue<WallTaskBehaviorData> commandListeningQueue = new ConcurrentListeningQueue<WallTaskBehaviorData>();
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
	private final ReferenceFrame chestFrame;
	private final ReferenceFrame handControlFrame;

	// Handpose commands
	private HandPosePacket straightLineControlCmd;
	private HandRotateAboutAxisPacket circleControlCmd;
	private final DoubleYoVariable straightTrajectoryTime;

	// Circle
	private final DoubleYoVariable circleTrajectorytime;
	private FramePoint rotationAxisOrigin = new FramePoint();
	private FrameVector rotationAxis = new FrameVector();
	private Vector3d startToCenter = new Vector3d(0.0, 0.0, 0.20);
	private int circlecounter = 0;
	// Straight lines
	private FramePose handPose = new FramePose();
	private FramePoint nextCoordinate = new FramePoint();
	private Quat4d nextOrientationInWorld = new Quat4d();
	
	private Vector3d tempVector = new Vector3d();
	private FramePoint tempFramePoint = new FramePoint();
	//TODO: Eventually add Integrator part on low level forearm controllers.
	private final static double EPSILON = 100.0; //5.0e-3;
	private final DoubleYoVariable distanceToGoal;

	private enum BehaviorStates {SET_STARTPOSITION, WAIT_FOR_INPUT, WAIT, INSERT_DRILL, SEND_CUT_COMMAND_TO_CONTROLLER, RETRACT_DRILL, GET_TO_DROP_POSITION, DONE};
	private EnumYoVariable<BehaviorStates> behaviorState;
	private BehaviorStates nextBehaviorState;

	private final RobotSide robotSide;
	private Matrix3d executionRotationMatrix;
	private Matrix3d dropRotationMatrix;
	
	private FramePoint startPosition;
	private FramePoint startCutPosition;
	private FrameVector drillInsertion;
	private FramePoint dropPosition;

	public ForceControlledWallTaskBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, HumanoidReferenceFrames referenceFrames, FullRobotModel fullRobotModel, DoubleYoVariable yoTime, YoGraphicsListRegistry yoGraphicsRegistry)
	{
		super(outgoingCommunicationBridge);
		this.fullrobotModel = fullRobotModel;
		this.yoTime = yoTime;
		this.visualizerYoGraphicsRegistry = yoGraphicsRegistry;
		this.robotSide = RobotSide.RIGHT;
		this.attachControllerListeningQueue(handStatusListeningQueue, HandPoseStatus.class);
		this.attachNetworkProcessorListeningQueue(commandListeningQueue, WallTaskBehaviorData.class);

		behaviorState = new EnumYoVariable<ForceControlledWallTaskBehavior.BehaviorStates>("ForceControlledWallTaskBehavior_State", registry, BehaviorStates.class, true);
		behaviorState.set(null);

		isDone = new BooleanYoVariable("isDone", registry);
		isDone.set(false);

		// Reference Frames:
		worldFrame = HumanoidReferenceFrames.getWorldFrame();
		chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
		handControlFrame = fullRobotModel.getHandControlFrame(robotSide);

		yoHandPose = new YoFramePose("yoHandPose", worldFrame, registry);
		yoTrajectoryControlFrame = new YoGraphicCoordinateSystem("yoTrajectoryControlFrame", yoHandPose, 0.2);
		visualizerYoGraphicsRegistry.registerYoGraphic("yoTrajectoryControlFrameViz", yoTrajectoryControlFrame);

		straightTrajectoryTime = new DoubleYoVariable("straightTrajectoryTime", registry);
		circleTrajectorytime = new DoubleYoVariable("circularTrajectoryTime", registry);
		//WARNING: INCREASE IF POSITION CONTROLLED IS APPLIED
		//circleTrajectorytime.set(1.0);
		circleTrajectorytime.set(1.0);
		
		distanceToGoal = new DoubleYoVariable(getName()+ "distanceToGoal", registry);
		straightLineControlCmd = new HandPosePacket(robotSide, Frame.WORLD, null, null, 1.0);
		circleControlCmd = new HandRotateAboutAxisPacket();

		rotationAxis = new FrameVector();

		executionRotationMatrix = new Matrix3d();
		
		dropRotationMatrix = new Matrix3d();
		
		
		startPosition = new FramePoint(chestFrame);
		startCutPosition = new FramePoint(chestFrame);
		dropPosition = new FramePoint(chestFrame);
		drillInsertion = new FrameVector(chestFrame, 0.15, 0.0, 0.0);
		
		// Set coordinates depending on robotSide
		executionRotationMatrix.rotZ(robotSide.negateIfLeftSide(Math.PI / 2.0));
		
		//Since hand mount for drill turn on is asymmetric
		if(robotSide == RobotSide.RIGHT)
		{
			Matrix3d xrotationMatrix = new Matrix3d();
			xrotationMatrix.rotX(Math.PI);
			executionRotationMatrix.mul(xrotationMatrix);
		}
		
		
		startPosition.set(0.5, robotSide.negateIfRightSide(0.05), -0.45);
		startCutPosition.set(startPosition);
		startCutPosition.add(drillInsertion);
		dropRotationMatrix.rotX(robotSide.negateIfLeftSide(Math.PI / 2.0));
		dropPosition.set(0.45, robotSide.negateIfRightSide(0.55), -0.65);
	}

	@Override
	public void doControl()
	{
		if (handStatusListeningQueue.isNewPacketAvailable())
		{
			consumeHandPoseStatus(handStatusListeningQueue.getNewestPacket());
		}
		
		if (commandListeningQueue.isNewPacketAvailable())
		{
			WallTaskBehaviorData packet = commandListeningQueue.getNewestPacket();
			if(packet.getCommand() == Commands.START)
			{
				behaviorState.set(BehaviorStates.SET_STARTPOSITION);
			}
			if(packet.getCommand() == Commands.INSERT)
			{
				behaviorState.set(BehaviorStates.INSERT_DRILL);
			}
			if(packet.getCommand() == Commands.CUT)
			{
				behaviorState.set(BehaviorStates.SEND_CUT_COMMAND_TO_CONTROLLER);
			}
		}

		switch(behaviorState.getEnumValue())
		{
		case SET_STARTPOSITION :
			goToStartPosition();
			nextBehaviorState = BehaviorStates.WAIT_FOR_INPUT;
			behaviorState.set(BehaviorStates.WAIT);
			break;
			
		case INSERT_DRILL :
			insertDrill();
			nextBehaviorState = BehaviorStates.WAIT_FOR_INPUT;
			behaviorState.set(BehaviorStates.WAIT);
			break;
			
		case RETRACT_DRILL :
			retractDrill();
			nextBehaviorState = BehaviorStates.GET_TO_DROP_POSITION;
			behaviorState.set(BehaviorStates.WAIT);
			break;

		case WAIT :
			tempFramePoint.setToZero(handControlFrame);
			tempFramePoint.changeFrame(worldFrame);
			tempVector.set(tempFramePoint.getPoint().x, tempFramePoint.getPoint().y, tempFramePoint.getPoint().z);
			if(nextCoordinate.getReferenceFrame() != worldFrame)
			{
				nextCoordinate.changeFrame(worldFrame);
			}
			
			tempVector.sub(nextCoordinate.getPoint());

			distanceToGoal.set(tempVector.length());

			if(distanceToGoal.getDoubleValue() < EPSILON && status == Status.COMPLETED)
			{
				behaviorState.set(nextBehaviorState);
				status = null;
			}
			break;

		case SEND_CUT_COMMAND_TO_CONTROLLER :
			sendCutCommand();
			circlecounter ++;
			if (circlecounter == 1)
			{
				nextBehaviorState = BehaviorStates.RETRACT_DRILL;
			}
			else
			{
				nextBehaviorState = BehaviorStates.SEND_CUT_COMMAND_TO_CONTROLLER;
			}
			behaviorState.set(BehaviorStates.WAIT);
			break;

		case GET_TO_DROP_POSITION :
			goToDropPosition();
			nextBehaviorState = BehaviorStates.DONE;
			behaviorState.set(BehaviorStates.WAIT);
			break;

		case DONE :
			isDone.set(true);
			break;
			
		case WAIT_FOR_INPUT :
			// Wait for input.
			break;

		default :
			PrintTools.error(this, "No valid behavior state. Set initial state.");
			break;
		}
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
	public void doPostBehaviorCleanup()
	{
		status = null;
		hasBeenInitialized.set(false);
		isDone.set(false);
		defaultPostBehaviorCleanup();
	}

	@Override
	public void initialize()
	{
		handPose.setToZero(chestFrame);
		handPose.setPosition(startPosition);
		
		handPose.setOrientation(executionRotationMatrix);

		rotationAxis.setToZero(chestFrame);
		rotationAxis.set(1.0, 0.0, 0.0);
		behaviorState.set(BehaviorStates.WAIT_FOR_INPUT);

		status = null;
		hasBeenInitialized.set(true);
	}
	
	private void insertDrill()
	{
		handPose.setToZero(chestFrame);
		handPose.setPosition(startCutPosition);
		handPose.setOrientation(executionRotationMatrix);
		
		handPose.changeFrame(worldFrame);
		handPose.getPositionIncludingFrame(nextCoordinate);
		handPose.getOrientation(nextOrientationInWorld);
		
		straightTrajectoryTime.set(5.0);
		
		moveInStraightLine(nextCoordinate.getPoint(), nextOrientationInWorld, straightTrajectoryTime.getDoubleValue());
		
	}
	
	private void retractDrill()
	{
		handPose.setToZero(handControlFrame);
		handPose.changeFrame(chestFrame);
		handPose.getPositionIncludingFrame(nextCoordinate);
		nextCoordinate.sub(drillInsertion);
		
		handPose.changeFrame(worldFrame);
		handPose.getOrientation(nextOrientationInWorld);
		
		nextCoordinate.changeFrame(worldFrame);
		straightTrajectoryTime.set(5.0);
		moveInStraightLine(nextCoordinate.getPoint(), nextOrientationInWorld, straightTrajectoryTime.getDoubleValue());
	}

	private void goToStartPosition()
	{
		handPose.changeFrame(worldFrame);
		handPose.getPositionIncludingFrame(nextCoordinate);
		handPose.getOrientation(nextOrientationInWorld);
		straightTrajectoryTime.set(10.0);
		
		moveInStraightLine(nextCoordinate.getPoint(), nextOrientationInWorld, straightTrajectoryTime.getDoubleValue());
	}
	
	private void sendCutCommand()
	{
		handPose.changeFrame(worldFrame);
		handPose.getPositionIncludingFrame(rotationAxisOrigin);
		rotationAxisOrigin.add(startToCenter);
		rotationAxis.changeFrame(worldFrame);
		
		initializeForceControlCircle(rotationAxisOrigin.getPoint(), rotationAxis.getVector(), -5.0, -3.0);
		sendPacketToController(circleControlCmd);
	}
	
	private void initializeForceControlCircle(Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld, double tangentialForce, double normalForce)
	{
		circleControlCmd.graspOffsetFromControlFrame = 0.0;
		circleControlCmd.robotSide = robotSide;
		circleControlCmd.trajectoryTime = circleTrajectorytime.getDoubleValue();
		circleControlCmd.controlHandAngleAboutAxis = false;
		circleControlCmd.rotationAxisInWorld = rotationAxisInWorld;
		circleControlCmd.rotationAxisOriginInWorld = rotationAxisOriginInWorld;
		circleControlCmd.rotationRightHandRule = 2.0 * Math.PI;

		Vector3d normalForceVector = new Vector3d(rotationAxisInWorld);
		normalForceVector.scale(normalForce);

		circleControlCmd.setForceControlParameters(tangentialForce, normalForceVector);
	}

	private void goToDropPosition()
	{
		handPose.setToZero(chestFrame);
		handPose.setOrientation(dropRotationMatrix);
		handPose.setPosition(dropPosition);

		handPose.changeFrame(worldFrame);
		handPose.getPositionIncludingFrame(nextCoordinate);
		handPose.getOrientation(nextOrientationInWorld);

		straightTrajectoryTime.set(10.0);
		
		moveInStraightLine(nextCoordinate.getPoint(), nextOrientationInWorld, straightTrajectoryTime.getDoubleValue());
	}

	private void moveInStraightLine(Point3d nextCoordinateInWorld, Quat4d nextOrientationInWorld, double trajectoryTime)
	{
		straightLineControlCmd.position = nextCoordinateInWorld;
		straightLineControlCmd.orientation = nextOrientationInWorld;
		straightLineControlCmd.trajectoryTime = trajectoryTime;
		sendPacketToController(straightLineControlCmd);
	}
	
	@Override
	public boolean hasInputBeenSet()
	{
		// TODO Auto-generated method stub
		return false;
	}
}

