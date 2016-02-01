package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataList;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPathPlanPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus.Status;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
/**
 * WalkToLocation with path-planning algorithm to avoid keepout regions
 * 
 * @author Alex Graber-Tilton
 *
 */
public class WalkToGoalBehavior extends BehaviorInterface {

	private final BooleanYoVariable DEBUG = new BooleanYoVariable("DEBUG", registry);
	private final DoubleYoVariable yoTime;
	private double searchStartTime = 0;

	private final ConcurrentListeningQueue<WalkToGoalBehaviorPacket> inputListeningQueue = new ConcurrentListeningQueue<WalkToGoalBehaviorPacket>();
	private final ConcurrentListeningQueue<FootstepPathPlanPacket> plannedPathListeningQueue = new ConcurrentListeningQueue<FootstepPathPlanPacket>();
	private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue  = new ConcurrentListeningQueue<FootstepStatus>();;
	private final BooleanYoVariable isDone;
	private final BooleanYoVariable hasInputBeenSet;
	private final FullHumanoidRobotModel fullRobotModel;

	private FootstepData startFootstep;
	private ArrayList<FootstepData> goalFootsteps = new ArrayList<FootstepData>();
	private double startYaw;

	private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

	private final BooleanYoVariable hasNewPlan = new BooleanYoVariable("hasNewPlan", registry);
	private final BooleanYoVariable stepCompleted = new BooleanYoVariable("stepCompleted", registry);
	private final BooleanYoVariable allStepsCompleted = new BooleanYoVariable("allStepsCompleted", registry);
   private final BooleanYoVariable requestQuickSearch = new BooleanYoVariable("requestQuickSearch", registry);
	private final BooleanYoVariable waitingForValidPlan = new BooleanYoVariable("waitingForValidPlan", registry);
	private final BooleanYoVariable executePlan = new BooleanYoVariable("executePlan", registry);
	private final BooleanYoVariable executeUnknownFirstStep = new BooleanYoVariable("executeUnknownFirstStep", registry);

//	private ArrayList<FootstepData> footsteps = new ArrayList<FootstepData>();
	
	private FootstepData currentLocation;
	private FootstepData predictedLocation;
	private FootstepPathPlanPacket currentPlan;
	private List<FootstepData> stepsRequested;
	private double ankleHeight = 0;
	private int expectedIndex = 0;
   private RobotSide lastSide = null;


	public WalkToGoalBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel, DoubleYoVariable yoTime, double ankleHeight)
	{
		super(outgoingCommunicationBridge);
		DEBUG.set(true);
		this.yoTime = yoTime;
		this.ankleHeight = ankleHeight;

		isDone = new BooleanYoVariable("isDone", registry);
		hasInputBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);

		this.fullRobotModel = fullRobotModel;

		this.attachNetworkProcessorListeningQueue(inputListeningQueue, WalkToGoalBehaviorPacket.class);
		this.attachNetworkProcessorListeningQueue(plannedPathListeningQueue, FootstepPathPlanPacket.class);
		this.attachControllerListeningQueue(footstepStatusQueue, FootstepStatus.class);
	}

	@Override
	public void doControl() {
		if (!isDone.getBooleanValue()){
			checkForNewInputs();
		}
		if (!hasInputBeenSet()){
			return;
		}
		if (atGoal()){
			debugPrintln("At goal, plan complete");
			hasInputBeenSet.set(false);
         executePlan.set(false);
			return;
		}
		if (checkForNewPlan()){
			return;
		}		
		if (checkForStepCompleted()){
			return;
		}

		if (executePlan.getBooleanValue() && hasNewPlan.getBooleanValue() && (!stepCompleted.getBooleanValue() || allStepsCompleted.getBooleanValue())){
			if (planValid(currentPlan) && (!currentPlan.footstepUnknown.get(1) || executeUnknownFirstStep.getBooleanValue())){
				processNextStep();
            return;
			}
		}

      if (currentPlan != null && currentPlan.footstepUnknown != null && (currentPlan.footstepUnknown.isEmpty() || currentPlan.footstepUnknown.get(1)) && requestQuickSearch.getBooleanValue()){
         //no plan or next step unknown, need a new plan fast
         debugPrintln("Quick Search Requested");
         requestFootstepPlan();
         requestQuickSearch.set(false);

      }
	}
	
	private boolean checkForNewPlan(){
		//return true if new packet, else return false.
		FootstepPathPlanPacket newestPacket = plannedPathListeningQueue.getNewestPacket();
		if (newestPacket != null){
			System.out.println("New plan received. Checking validity...");
			if (planValid(newestPacket)){
				currentPlan = newestPacket;
				debugPrintln("Valid plan, new plan is:");
				debugPrintln(currentPlan.pathPlan.toString());
				hasNewPlan.set(true);
				//stop current steps
				visualizePlan(currentPlan);
			}else{
//				visualizePlan(newestPacket);
				System.out.println("Plan is not Valid!");
			}
			return true;
		}		
		return false;
	}
	
	private void visualizePlan(FootstepPathPlanPacket plan){
		if (plan.pathPlan == null || plan.pathPlan.size() == 0) return;
		int size = plan.pathPlan.size();
		SnapFootstepPacket planVisualizationPacket = new SnapFootstepPacket();
		planVisualizationPacket.footstepData = new ArrayList<FootstepData>();
		planVisualizationPacket.footstepOrder = new int[size];
		planVisualizationPacket.flag = new byte[size];
		
		for (int i = 0; i < size; i++){
			planVisualizationPacket.footstepData.add(adjustFootstepForAnkleHeight(plan.pathPlan.get(i)));
			planVisualizationPacket.footstepOrder[i] = i;
			planVisualizationPacket.flag[i] = (byte) (plan.footstepUnknown.get(i) ? 0 : 2);
		}
		planVisualizationPacket.setDestination(PacketDestination.NETWORK_PROCESSOR);
		sendPacketToNetworkProcessor(planVisualizationPacket);
	}
	
	private boolean planValid(FootstepPathPlanPacket plan){
		if (plan == null) return false;
		if (!plan.goalsValid) return false;
		for (int i=0; i < plan.originalGoals.size(); i++){
			if (!approximatelyEqual(plan.originalGoals.get(i),goalFootsteps.get(i))) return false;
		}
		while (plan.pathPlan.size() > 0 && !approximatelyEqual(plan.pathPlan.get(0), predictedLocation)){
			plan.pathPlan.remove(0);
			plan.footstepUnknown.remove(0);
		}
		if (plan.pathPlan.size() < 2) return false;
		if (approximatelyEqual(plan.pathPlan.get(1), currentLocation)) return false;
		waitingForValidPlan.set(false);
		return true;
	}
	
	private boolean checkForStepCompleted(){
		//return true if there was a new packet, otherwise return false.
		FootstepStatus newestPacket = footstepStatusQueue.getNewestPacket();
		if (newestPacket != null){
			//TODO: update current location and predicted location from the feedback
			if (newestPacket.status == Status.STARTED){ 
				stepCompleted.set(false);
				debugPrintln("Number of requested steps: " + stepsRequested.size());
				debugPrintln("footstep index: " + newestPacket.footstepIndex);
				debugPrintln("expected index: " + expectedIndex);
				predictedLocation = stepsRequested.get(expectedIndex);
				debugPrintln("Predicted now at " + predictedLocation.toString());
				sendUpdateStart(predictedLocation);
				expectedIndex++;
			}else if (newestPacket.status == Status.COMPLETED){
				stepCompleted.set(true);
				currentLocation = predictedLocation;
            FootstepData actualFootstep = new FootstepData(newestPacket.getRobotSide(), newestPacket.getActualFootPositionInWorld(), newestPacket.getActualFootOrientationInWorld());
            lastSide = newestPacket.getRobotSide();

            debugPrintln("Step Completed, expected location is: "+ currentLocation.toString());
            debugPrintln("Step Completed, actual location is: "+ actualFootstep.toString());
				if (newestPacket.footstepIndex == stepsRequested.size()-1){
					debugPrintln("All steps complete");
					allStepsCompleted.set(true);
               requestQuickSearch.set(true);
				}
			}
			return true;
		}
		return false;
	}
	
	private void processNextStep(){
		if (!planValid(currentPlan)){
			debugPrintln("current plan is invalid, waiting for new plan");
			if (!waitingForValidPlan.getBooleanValue()){
				sendUpdateStart(predictedLocation);
				waitingForValidPlan.set(true);
			}
		}else{
			takeStep();
			hasNewPlan.set(false);
		}
	}

	private void takeStep(){
		//remove current location from plan, element 1 is next step
		currentPlan.pathPlan.remove(0);
		currentPlan.footstepUnknown.remove(0);
		//element 1 is now element 0
		if (currentPlan.footstepUnknown.get(0) && !executeUnknownFirstStep.getBooleanValue()) return;
		
		sendStepsToController();
		stepCompleted.set(false);
		
	}
	
	private boolean atGoal(){
		if (currentLocation == null) return false;
		for (FootstepData goal : goalFootsteps){
			if (approximatelyEqual(currentLocation, goal)){
				hasInputBeenSet.set(false);
				return true;
			}
		}
		return false;
	}

	private boolean approximatelyEqual(FootstepData currentLocation, FootstepData checkAgainst){
		if (currentLocation == null) return false;
		double xDiff = currentLocation.location.x - checkAgainst.location.x;
		double yDiff = currentLocation.location.y - checkAgainst.location.y;
		if (currentLocation.robotSide != checkAgainst.robotSide) return false;
		if (Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff,2)) > 0.05) return false;
		if (Math.abs(RotationTools.computeYaw(currentLocation.orientation) - RotationTools.computeYaw(checkAgainst.orientation)) > Math.PI/16) return false;
		return true;
	}

	private void requestFootstepPlan()
	{
		FootstepPlanRequestPacket footstepPlanRequestPacket = new FootstepPlanRequestPacket(FootstepPlanRequestPacket.RequestType.START_SEARCH, startFootstep,startYaw,goalFootsteps, 10);
		outgoingCommunicationBridge.sendPacketToNetworkProcessor(footstepPlanRequestPacket);
		waitingForValidPlan.set(true);
	}

	private void requestSearchStop(){
		FootstepPlanRequestPacket stopSearchRequestPacket = new FootstepPlanRequestPacket(FootstepPlanRequestPacket.RequestType.STOP_SEARCH,new FootstepData(), 0.0, null);
		outgoingCommunicationBridge.sendPacketToNetworkProcessor(stopSearchRequestPacket);
		waitingForValidPlan.set(false);
	}
	
	private void sendUpdateStart(FootstepData updatedLocation){
		if (updatedLocation.orientation.epsilonEquals(new Quat4d(), .003)) return;
		FootstepPlanRequestPacket updateStartPacket = new FootstepPlanRequestPacket(FootstepPlanRequestPacket.RequestType.UPDATE_START, updatedLocation, RotationTools.computeYaw(updatedLocation.orientation), null, 10);
		outgoingCommunicationBridge.sendPacketToNetworkProcessor(updateStartPacket);
	}
	
	private void sendStepsToController(){
		int size = currentPlan.footstepUnknown.size();
		FootstepDataList outgoingFootsteps = new FootstepDataList();
		for (int i = 0; i < size; i ++){
         if (executeUnknownFirstStep.getBooleanValue() && i ==0){
            outgoingFootsteps.footstepDataList.add(adjustFootstepForAnkleHeight(currentPlan.pathPlan.get(i)));
            executeUnknownFirstStep.set(false);
         }else	if (!currentPlan.footstepUnknown.get(i)){
				outgoingFootsteps.footstepDataList.add(adjustFootstepForAnkleHeight(currentPlan.pathPlan.get(i)));
			}else{
				break;
			}
         debugPrintln("Step Added To Send: " + outgoingFootsteps.footstepDataList.get(i));
         if (outgoingFootsteps.footstepDataList.get(i).predictedContactPoints != null && outgoingFootsteps.footstepDataList.get(i).predictedContactPoints.isEmpty()){
            debugPrintln("Support Points are: " + outgoingFootsteps.footstepDataList.get(i).predictedContactPoints.toString());
            throw new RuntimeException("attempting to send footstep with empty list of support points");
         }
		}
		stepsRequested = outgoingFootsteps.footstepDataList;
		debugPrintln(stepsRequested.size() + " steps sent to controller");
		debugPrintln(stepsRequested.toString());
		outgoingFootsteps.setDestination(PacketDestination.CONTROLLER);
		allStepsCompleted.set(false);
      sendPacketToController(outgoingFootsteps);
      expectedIndex = 0;
    }
	
	private FootstepData adjustFootstepForAnkleHeight(FootstepData footstep){
		FootstepData copy = new FootstepData(footstep);
		Point3d ankleOffset = new Point3d(0, 0, ankleHeight);
		RigidBodyTransform footTransform = new RigidBodyTransform();
		footTransform.setRotationAndZeroTranslation(copy.getOrientation());
		footTransform.transform(ankleOffset);
		copy.getLocation().add(ankleOffset);
		return copy;
	}
	
	private void checkForNewInputs()
	{
		WalkToGoalBehaviorPacket newestPacket = inputListeningQueue.getNewestPacket();
		if (newestPacket != null)
		{
         if (newestPacket.action == WalkToGoalBehaviorPacket.WalkToGoalAction.FIND_PATH){
            set(newestPacket.getGoalPosition()[0], newestPacket.getGoalPosition()[1], newestPacket.getGoalPosition()[2], newestPacket.getGoalSide());
            requestFootstepPlan();
            hasInputBeenSet.set(true);
            debugPrintln("Requesting path");
         }else	if (newestPacket.action == WalkToGoalBehaviorPacket.WalkToGoalAction.EXECUTE){
				debugPrintln("Executing path");
            sendPacketToController(new PauseCommand(false));
				executePlan.set(true);
			}else if (newestPacket.action == WalkToGoalBehaviorPacket.WalkToGoalAction.EXECUTE_UNKNOWN){
            executeUnknownFirstStep.set(true);
            debugPrintln("First step now allowed to be unknown");
         }else if (newestPacket.action == WalkToGoalBehaviorPacket.WalkToGoalAction.STOP){
            executePlan.set(false);
            sendPacketToController(new PauseCommand(true));
            debugPrintln("Stopping execution");
         }
		}
	}

	public void set(double xGoal, double yGoal, double thetaGoal, RobotSide goalSide)
	{
      RobotSide startSide;
		if (lastSide == null) {
         startSide = RobotSide.LEFT;
      }else{
         startSide = lastSide;
      }
		Vector3d startTranslation = new Vector3d();
		Matrix3d startRotation = new Matrix3d();
		fullRobotModel.getFoot(startSide).getBodyFixedFrame().getTransformToDesiredFrame(worldFrame).getTranslation(startTranslation);
		fullRobotModel.getFoot(startSide).getBodyFixedFrame().getTransformToDesiredFrame(worldFrame).getRotation(startRotation);
		startYaw = RotationTools.computeYaw(startRotation);
		Quat4d startOrientation = new Quat4d();
		RotationTools.convertMatrixToQuaternion(startRotation, startOrientation);
		startFootstep = new FootstepData(startSide, new Point3d(startTranslation), startOrientation);
		currentLocation = new FootstepData(startFootstep);
		predictedLocation = currentLocation;
		stepCompleted.set(true);

		double robotYOffset = 0.1;
		double xOffset = -1 * robotYOffset * Math.sin(thetaGoal);
		double yOffset = robotYOffset * Math.cos(thetaGoal);

		goalFootsteps.clear();
		//set left foot goal 
      /*
		Quat4d leftGoalOrientation = new Quat4d();
		RotationFunctions.setQuaternionBasedOnYawPitchRoll(leftGoalOrientation, thetaGoal, 0,0);
		goalFootsteps.add(new FootstepData(RobotSide.LEFT, new Point3d(xGoal + xOffset, yGoal + yOffset, 0), leftGoalOrientation));
      */

		Quat4d rightGoalOrientation = new Quat4d();
		RotationTools.convertYawPitchRollToQuaternion(thetaGoal, 0, 0,rightGoalOrientation);
		goalFootsteps.add(new FootstepData(RobotSide.RIGHT, new Point3d(xGoal - xOffset, yGoal - yOffset, 0), rightGoalOrientation));

		hasInputBeenSet.set(true);
	}

	@Override
	public void initialize()
	{
		stepCompleted.set(true);
		hasNewPlan.set(false);
		waitingForValidPlan.set(false);
		hasInputBeenSet.set(false);
		executePlan.set(false);
		executeUnknownFirstStep.set(false);
		allStepsCompleted.set(true);
      requestQuickSearch.set(false);
	}

	@Override
	protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
	{
	}

	@Override
	protected void passReceivedControllerObjectToChildBehaviors(Object object)
	{
	}

	@Override
	public void stop()
	{
		requestSearchStop();
      sendPacketToController(new PauseCommand(true));
		waitingForValidPlan.set(false);
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

      isPaused.set(true);
      sendPacketToController(new PauseCommand(true));
	}

	@Override
	public void resume()
	{
		isPaused.set(false);
      sendPacketToController(new PauseCommand(false));
	}

	@Override
	public boolean isDone()
	{
		return atGoal();
	}

	@Override
	public void doPostBehaviorCleanup()
	{
		isPaused.set(false);
		isStopped.set(false);
		stepCompleted.set(true);
		hasNewPlan.set(false);
		waitingForValidPlan.set(false);
		executePlan.set(false);
		executeUnknownFirstStep.set(false);
		requestSearchStop();
		hasInputBeenSet.set(false);
		allStepsCompleted.set(true);
      requestQuickSearch.set(false);
	}

	public boolean hasInputBeenSet()
	{
		return hasInputBeenSet.getBooleanValue();
	}
	
	public void debugPrintln(String string){
		if (DEBUG.getBooleanValue()) System.out.println(string);
	}
}
