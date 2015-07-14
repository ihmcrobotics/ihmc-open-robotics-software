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
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
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

	// Circle commands
	private final DoubleYoVariable circleTrajectorytime;
	private FrameVector rotationAxis;
	private Point3d rotationAxisOriginInWorld;
	private Vector3d rotationAxisInWorld;
	private Vector3d startToCenter = new Vector3d(0.0, 0.0, 0.20);

	private FramePose handPose = new FramePose();
	private Point3d nextCoordinate = new Point3d();
	private Quat4d nextOrientation = new Quat4d();

	private Vector3d tempVector = new Vector3d();
	private FramePoint tempFramePoint = new FramePoint();
	private final static double EPSILON = 5.0e-3;
	private final DoubleYoVariable distanceToGoal;

	private enum BehaviorStates {SET_STARTPOSITION, WAIT, SEND_CUT_COMMAND_TO_CONTROLLER, GET_TO_DROP_POSITION, DONE};
	private EnumYoVariable<BehaviorStates> behaviorState;
	private BehaviorStates nextBehaviorState;

	private final RobotSide robotSide;
	private Matrix3d executionRotationMatrix;
	private Matrix3d dropRotationMatrix;
	private Vector3d startPositionInChest;
	private Vector3d dropPositionInChest;


	public ForceControlledWallTaskBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, ReferenceFrames referenceFrames, FullRobotModel fullRobotModel, DoubleYoVariable yoTime, YoGraphicsListRegistry yoGraphicsRegistry)
	{
		super(outgoingCommunicationBridge);
		this.fullrobotModel = fullRobotModel;
		this.yoTime = yoTime;
		this.visualizerYoGraphicsRegistry = yoGraphicsRegistry;
		this.robotSide = RobotSide.LEFT;
		this.attachControllerListeningQueue(handStatusListeningQueue, HandPoseStatus.class);

		behaviorState = new EnumYoVariable<ForceControlledWallTaskBehavior.BehaviorStates>("ForceControlledWallTaskBehavior_State", registry, BehaviorStates.class, true);
		behaviorState.set(null);

		isDone = new BooleanYoVariable("isDone", registry);
		isDone.set(false);

		// Reference Frames:
		worldFrame = ReferenceFrames.getWorldFrame();
		chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
		handControlFrame = fullRobotModel.getHandControlFrame(robotSide);

		yoHandPose = new YoFramePose("yoHandPose", worldFrame, registry);
		yoTrajectoryControlFrame = new YoGraphicCoordinateSystem("yoTrajectoryControlFrame", yoHandPose, 0.2);
		visualizerYoGraphicsRegistry.registerYoGraphic("yoTrajectoryControlFrameViz", yoTrajectoryControlFrame);

		straightTrajectoryTime = new DoubleYoVariable("straightTrajectoryTime", registry);
		circleTrajectorytime = new DoubleYoVariable("circularTrajectoryTime", registry);
		circleTrajectorytime.set(20.0);

		distanceToGoal = new DoubleYoVariable(getName()+ "distanceToGoal", registry);
		straightLineControlCmd = new HandPosePacket(robotSide, Frame.WORLD, null, null, 1.0);
		circleControlCmd = new HandRotateAboutAxisPacket();
		rotationAxisInWorld = new Vector3d();
		rotationAxisOriginInWorld = new Point3d();

		rotationAxis = new FrameVector();

		executionRotationMatrix = new Matrix3d();
		dropRotationMatrix = new Matrix3d();
		startPositionInChest = new Vector3d();
		dropPositionInChest = new Vector3d();
		if(robotSide == RobotSide.RIGHT)
		{
			executionRotationMatrix.rotZ(Math.PI / 2.0);
			startPositionInChest.set(0.6, -0.05, -0.45);
			dropRotationMatrix.rotX(-Math.PI / 2.0);
			dropPositionInChest.set(0.45, -0.55, -0.65);
		}
		else if(robotSide == RobotSide.LEFT)
		{
			executionRotationMatrix.rotZ(-Math.PI / 2.0);
			startPositionInChest.set(0.6, +0.05, -0.45);
			dropRotationMatrix.rotX(Math.PI / 2.0);
			dropPositionInChest.set(0.45, +0.55, -0.65);
		}
		else
		{
			PrintTools.error(this, "robotSide not defined.");
		}
	}

	@Override
	public void doControl()
	{
		if (handStatusListeningQueue.isNewPacketAvailable())
		{
			consumeHandPoseStatus(handStatusListeningQueue.getNewestPacket());
		}

		switch(behaviorState.getEnumValue())
		{
		case SET_STARTPOSITION :
			goToStartPosition();
			nextBehaviorState = BehaviorStates.SEND_CUT_COMMAND_TO_CONTROLLER;
			behaviorState.set(BehaviorStates.WAIT);
			break;

		case WAIT :
			tempFramePoint.setToZero(handControlFrame);
			tempFramePoint.changeFrame(worldFrame);
			tempVector.set(tempFramePoint.getPoint().x, tempFramePoint.getPoint().y, tempFramePoint.getPoint().z);
			tempVector.sub(nextCoordinate);

			distanceToGoal.set(tempVector.length());

			if(distanceToGoal.getDoubleValue() < EPSILON && status == Status.COMPLETED)
			{
				behaviorState.set(nextBehaviorState);
				status = null;
			}
			break;

		case SEND_CUT_COMMAND_TO_CONTROLLER :
			sendCutCommand();
			nextBehaviorState = BehaviorStates.GET_TO_DROP_POSITION;
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
		handPose.setPosition(startPositionInChest);
		handPose.setOrientation(executionRotationMatrix);

		rotationAxis.setToZero(chestFrame);
		rotationAxis.set(1.0, 0.0, 0.0);
		behaviorState.set(BehaviorStates.SET_STARTPOSITION);

		status = null;
		hasBeenInitialized.set(true);
	}

	private void goToStartPosition()
	{
		handPose.changeFrame(worldFrame);
		handPose.getPosition(nextCoordinate);
		handPose.getOrientation(nextOrientation);
		straightTrajectoryTime.set(5.0);
		
		straightLineControlCmd.position = nextCoordinate;
		straightLineControlCmd.orientation = nextOrientation;
		straightLineControlCmd.trajectoryTime = straightTrajectoryTime.getDoubleValue();
		sendPacketToController(straightLineControlCmd);
	}
	
	private void sendCutCommand()
	{
		handPose.changeFrame(worldFrame);
		handPose.getPosition(rotationAxisOriginInWorld);
		rotationAxisOriginInWorld.add(startToCenter);
		rotationAxis.changeFrame(worldFrame);
		rotationAxisInWorld.set(rotationAxis.getVector());
		initializeForceControlCircle(rotationAxisOriginInWorld, rotationAxisInWorld, -5.0, -3.0);
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
		handPose.setPosition(dropPositionInChest);

		handPose.changeFrame(worldFrame);
		handPose.getPosition(nextCoordinate);
		handPose.getOrientation(nextOrientation);

		straightTrajectoryTime.set(5.0);
		straightLineControlCmd.trajectoryTime = straightTrajectoryTime.getDoubleValue();
		straightLineControlCmd.position = nextCoordinate;
		straightLineControlCmd.orientation = nextOrientation;
		straightLineControlCmd.trajectoryTime = straightTrajectoryTime.getDoubleValue();
		sendPacketToController(straightLineControlCmd);
	}

	@Override
	public boolean hasInputBeenSet()
	{
		// TODO Auto-generated method stub
		return false;
	}
}

