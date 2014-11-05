package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.communication.packets.walking.PlannedPathPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.planning.FootstepPlanState;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WalkToGoalBehavior extends BehaviorInterface {
	
	private final BooleanYoVariable DEBUG = new BooleanYoVariable("DEBUG", registry);
	
	private final ConcurrentListeningQueue<WalkToGoalBehaviorPacket> inputListeningQueue = new ConcurrentListeningQueue<WalkToGoalBehaviorPacket>();
	private final ConcurrentListeningQueue<PlannedPathPacket> plannedPathListeningQueue = new ConcurrentListeningQueue<PlannedPathPacket>();
	private final BooleanYoVariable isDone;
	private final BooleanYoVariable hasInputBeenSet;
	private final FullRobotModel fullRobotModel;
	
	private FootstepPlanState startState;
	private FootstepPlanState goalState;
	
	private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
	
	private final BooleanYoVariable hasTargetBeenProvided = new BooleanYoVariable("hasTargetBeenProvided", registry);
	private final BooleanYoVariable hasSearchRequestBeenSent = new BooleanYoVariable("hasSearchRequestBeenSent", registry);
	private final BooleanYoVariable hasNewFootsteps = new BooleanYoVariable("hasNewFootsteps", registry);;
	
	private ArrayList<FootstepData> footsteps = new ArrayList<FootstepData>();
	private FootstepListBehavior footstepListBehavior;
	
	public WalkToGoalBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
	{
		super(outgoingCommunicationBridge);
		DEBUG.set(true);
		
		isDone = new BooleanYoVariable("isDone", registry);
		hasInputBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);
		footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge);
		
		this.fullRobotModel = fullRobotModel;
		
		this.attachNetworkProcessorListeningQueue(inputListeningQueue, WalkToGoalBehaviorPacket.class);
		this.attachNetworkProcessorListeningQueue(plannedPathListeningQueue, PlannedPathPacket.class);
	}
	
	@Override
	public void doControl() {
		if (!isDone.getBooleanValue()){
			checkForNewInputs();
		}
		if (!hasInputBeenSet()){
			return;
		}
		if (!hasSearchRequestBeenSent.getBooleanValue())
		{
			requestFootstepPlan(); //Should be Single Iteration of Search to be able to interrupt and/or pause.
		}
		else {
			checkForPlannedPath();
			if (hasNewFootsteps.getBooleanValue())
			{
				FootstepDataList footsepDataList = new FootstepDataList();
				footsepDataList.footstepDataList = footsteps;
				footstepListBehavior.set(footsepDataList);
				hasNewFootsteps.set(false);
			}
			else
			{
				footstepListBehavior.doControl();
			}
		}
	}

	private void requestFootstepPlan()
	{
		FootstepPlanRequestPacket footstepPlanRequestPacket = new FootstepPlanRequestPacket(FootstepPlanRequestPacket.RequestType.START_SEARCH, startState.x, startState.y, startState.theta, startState.side, goalState.x, goalState.y, goalState.theta, goalState.side);
		outgoingCommunicationBridge.sendPacketToNetworkProcessor(footstepPlanRequestPacket);
		hasSearchRequestBeenSent.set(true);
	}

	private void checkForNewInputs()
	{
		WalkToGoalBehaviorPacket newestPacket = inputListeningQueue.getNewestPacket();
		if (newestPacket != null)
		{
			set(newestPacket.getGoalPosition()[0], newestPacket.getGoalPosition()[1], newestPacket.getGoalPosition()[2], newestPacket.getGoalSide());
			hasSearchRequestBeenSent.set(false);
		}
	}
	
	private void checkForPlannedPath()
	{
		PlannedPathPacket newestPacket = plannedPathListeningQueue.getNewestPacket();
		if (newestPacket != null)
		{
			footsteps = newestPacket.getPlannedPath();
			hasNewFootsteps.set(true);
		}
	}
	
	public void set(double xGoal, double yGoal, double thetaGoal, RobotSide goalSide)
	{
		RobotSide startSide = RobotSide.LEFT;
		Vector3d startTranslation = new Vector3d();
    	Matrix3d startRotation = new Matrix3d();
		fullRobotModel.getFoot(startSide).getBodyFixedFrame().getTransformToDesiredFrame(worldFrame).getTranslation(startTranslation);
		fullRobotModel.getFoot(startSide).getBodyFixedFrame().getTransformToDesiredFrame(worldFrame).getRotation(startRotation);
		double thetaStart = RotationFunctions.getYaw(startRotation);
		double xStart = startTranslation.x;
		double yStart = startTranslation.y;
		startState = new FootstepPlanState(xStart,yStart,thetaStart,startSide);
		goalState = new FootstepPlanState(xGoal, yGoal, thetaGoal, goalSide);
		hasInputBeenSet.set(true);
	}
	
	@Override
	public void initialize()
	{
		hasInputBeenSet.set(false);
		hasSearchRequestBeenSent.set(false);
		hasNewFootsteps.set(false);
		footstepListBehavior.initialize();
		startState = null;
		goalState = null;
	}
	
	@Override
	protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
	{
		if (footstepListBehavior != null)
			footstepListBehavior.consumeObjectFromNetworkProcessor(object);
	}
	
	@Override
	protected void passReceivedControllerObjectToChildBehaviors(Object object)
	{
		if (footstepListBehavior != null)
			footstepListBehavior.consumeObjectFromController(object);
	}
	
	@Override
	public void stop()
	{
		isStopped.set(true);
	}
	
	@Override
	public void enableActions()
	{
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public void pause()
	{
		footstepListBehavior.pause();
		isPaused.set(true);
	}
	
	@Override
	public void resume()
	{
		footstepListBehavior.resume();
		isPaused.set(false);
		
	}
	
	@Override
	public boolean isDone()
	{
		if (!hasNewFootsteps.getBooleanValue() || !hasTargetBeenProvided.getBooleanValue())
			return false;
		return footstepListBehavior.isDone();
	}
	
	@Override
	public void finalize()
	{
		isPaused.set(false);
		isStopped.set(false);
		hasTargetBeenProvided.set(false);
		hasNewFootsteps.set(false);
		hasSearchRequestBeenSent.set(false);
		footstepListBehavior.finalize();
	}
	
	public boolean hasInputBeenSet()
	{
		return hasInputBeenSet.getBooleanValue();
	}
}
