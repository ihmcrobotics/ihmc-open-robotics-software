package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.communication.packets.sensing.RequestElevationMapPacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.planning.configurationSpace.CostFunction;
import us.ihmc.planning.footstepPlan.AtlasFootstepParameters;
import us.ihmc.planning.footstepPlan.AtlasFootstepPlanState;
import us.ihmc.planning.footstepPlan.AtlasManeuverabilityFunction;
import us.ihmc.planning.footstepPlan.AtlasPlanCostFunction;
import us.ihmc.planning.plan.astar.AStar;
import us.ihmc.planning.plan.astar.maneuverability.ManeuverabilityFunction;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WalkToGoalBehavior extends BehaviorInterface {
	
	private final BooleanYoVariable DEBUG = new BooleanYoVariable("DEBUG", registry);
	
	private final ConcurrentListeningQueue<WalkToGoalBehaviorPacket> inputListeningQueue = new ConcurrentListeningQueue<WalkToGoalBehaviorPacket>();
	private final BooleanYoVariable isDone;
	private final BooleanYoVariable hasInputBeenSet;
	private final FullRobotModel fullRobotModel;
	
	private AtlasFootstepPlanState startState;
	private AtlasFootstepPlanState goalState;
	
	private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
	
	private final BooleanYoVariable hasTargetBeenProvided = new BooleanYoVariable("hasTargetBeenProvided", registry);
	private final BooleanYoVariable hasFootstepsBeenGenerated = new BooleanYoVariable("hasFootstepsBeenGenerated", registry);
	private final BooleanYoVariable hasSearchFinished = new BooleanYoVariable("hasSearchFinished", registry);
	private final BooleanYoVariable hasNewFootsteps = new BooleanYoVariable("hasNewFootsteps", registry);;
	
	
	private ArrayList<FootstepData> footsteps = new ArrayList<FootstepData>();
	private FootstepListBehavior footstepListBehavior;
	
	private Object map = new Object();
	private final AtlasFootstepParameters footstepParameters = new AtlasFootstepParameters();
	private final CostFunction<AtlasFootstepPlanState> costFunction = new AtlasPlanCostFunction(map);
	private final ManeuverabilityFunction<AtlasFootstepPlanState> maneuverabilityFunction = new AtlasManeuverabilityFunction(map, footstepParameters);
	private final AStar<AtlasFootstepPlanState> atlasPathFinder = new AStar<AtlasFootstepPlanState>(costFunction, maneuverabilityFunction);
	
	public WalkToGoalBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
	{
		super(outgoingCommunicationBridge);
		DEBUG.set(true);
		
		isDone = new BooleanYoVariable("isDone", registry);
		hasInputBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);
		footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge);
		
		this.fullRobotModel = fullRobotModel;
		this.attachNetworkProcessorListeningQueue(inputListeningQueue, WalkToGoalBehaviorPacket.class);
	}
	
	@Override
	public void doControl() {
		if (!isDone.getBooleanValue()){
			checkForNewInputs();
		}
			
		if (!hasFootstepsBeenGenerated.getBooleanValue() && hasInputBeenSet())
		{
	        generateFootsteps(); //Should be Single Iteration of Search to be able to interrupt and/or pause.
		}
		else if (hasNewFootsteps.getBooleanValue())
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
	
	private void generateFootsteps() {
		//Do Search
		atlasPathFinder.setCumulativeCostValid(true);
		atlasPathFinder.USE_CONSIDERED_NODES = false;

		System.out.println("Starting location is: " + startState.toString());
		System.out.println("Goal location is: " + goalState.toString());

		List<AtlasFootstepPlanState> footPath = atlasPathFinder.computePath(startState, goalState);
		if (footPath.size() == 0)
		{
			System.out.println("No path found from " + startState + " to " + goalState);
		}

		if (DEBUG.getBooleanValue())
		{
			for (AtlasFootstepPlanState state : footPath)
			{
				System.out.println(state.toString());
			}
		}
		
		ArrayList<FootstepData> footstepData = generateFootstepList(footPath);
		hasSearchFinished.set(true);
				
		if (hasSearchFinished.getBooleanValue()){
			//footsteps = null; 
			hasFootstepsBeenGenerated.set(true);
			if (footstepData != footsteps){ //TODO: Change so it actually checks values, not the reference
				footsteps = footstepData;
				hasNewFootsteps.set(true);
			}
		}
	}
	
	public ArrayList<FootstepData> generateFootstepList(List<AtlasFootstepPlanState> footstepLocations )
	{
		ArrayList<FootstepData> footsteps = new ArrayList<FootstepData>();
		for (AtlasFootstepPlanState footstep : footstepLocations)
		{
			Quat4d orientation = new Quat4d();
			RotationFunctions.setQuaternionBasedOnYawPitchRoll(orientation, footstep.theta, 0, 0);
			FootstepData current = new FootstepData(footstep.getRobotSide(), footstep.getPoint3d(), orientation);
			footsteps.add(current);
		}
		return footsteps;
	}

	private void checkForNewInputs()
	{
		WalkToGoalBehaviorPacket newestPacket = inputListeningQueue.getNewestPacket();
		if (newestPacket != null)
		{
			set(newestPacket.getGoalPosition()[0], newestPacket.getGoalPosition()[1], newestPacket.getGoalPosition()[2], newestPacket.getGoalSide());
			RequestElevationMapPacket requestMap = new RequestElevationMapPacket(0,0,6,6);
			outgoingCommunicationBridge.sendPacketToNetworkProcessor(requestMap);
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
		startState = new AtlasFootstepPlanState(xStart,yStart,thetaStart,startSide);
		goalState = new AtlasFootstepPlanState(xGoal, yGoal, thetaGoal, goalSide);
		hasInputBeenSet.set(true);
	}
	
	@Override
	public void initialize()
	{
		hasInputBeenSet.set(false);
		hasFootstepsBeenGenerated.set(false);
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
		if (!hasFootstepsBeenGenerated.getBooleanValue() || !hasTargetBeenProvided.getBooleanValue())
			return false;
		return footstepListBehavior.isDone();
	}
	
	@Override
	public void finalize()
	{
		isPaused.set(false);
		isStopped.set(false);
		hasTargetBeenProvided.set(false);
		hasFootstepsBeenGenerated.set(false);
		footstepListBehavior.finalize();
	}
	
	public boolean hasInputBeenSet()
	{
		return hasInputBeenSet.getBooleanValue();
	}
}
