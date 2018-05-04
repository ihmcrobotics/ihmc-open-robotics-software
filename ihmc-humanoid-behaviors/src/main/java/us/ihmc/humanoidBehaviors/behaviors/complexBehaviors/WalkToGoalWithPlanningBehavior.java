package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPathPlanPacket;
import controller_msgs.msg.dds.FootstepPlanRequestPacket;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.SnapFootstepPacket;
import controller_msgs.msg.dds.WalkToGoalBehaviorPacket;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalAction;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestType;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * WalkToLocation with path-planning algorithm to avoid keepout regions
 *
 * @author Alex Graber-Tilton
 *
 */
public class WalkToGoalWithPlanningBehavior extends AbstractBehavior
{

   private final YoBoolean DEBUG = new YoBoolean("DEBUG", registry);
   private final YoDouble yoTime;
   private double searchStartTime = 0;

   private final ConcurrentListeningQueue<WalkToGoalBehaviorPacket> inputListeningQueue = new ConcurrentListeningQueue<WalkToGoalBehaviorPacket>(20);
   private final ConcurrentListeningQueue<FootstepPathPlanPacket> plannedPathListeningQueue = new ConcurrentListeningQueue<FootstepPathPlanPacket>(20);
   private final ConcurrentListeningQueue<FootstepStatusMessage> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatusMessage>(100);
   private final YoBoolean isDone;
   private final YoBoolean hasInputBeenSet;
   private final FullHumanoidRobotModel fullRobotModel;

   private FootstepDataMessage startFootstep;
   private ArrayList<FootstepDataMessage> goalFootsteps = new ArrayList<FootstepDataMessage>();
   private double startYaw;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoBoolean hasNewPlan = new YoBoolean("hasNewPlan", registry);
   private final YoBoolean stepCompleted = new YoBoolean("stepCompleted", registry);
   private final YoBoolean allStepsCompleted = new YoBoolean("allStepsCompleted", registry);
   private final YoBoolean requestQuickSearch = new YoBoolean("requestQuickSearch", registry);
   private final YoBoolean waitingForValidPlan = new YoBoolean("waitingForValidPlan", registry);
   private final YoBoolean executePlan = new YoBoolean("executePlan", registry);
   private final YoBoolean executeUnknownFirstStep = new YoBoolean("executeUnknownFirstStep", registry);

   //	private ArrayList<FootstepData> footsteps = new ArrayList<FootstepData>();

   private FootstepDataMessage currentLocation;
   private FootstepDataMessage predictedLocation;
   private FootstepPathPlanPacket currentPlan;
   private List<FootstepDataMessage> stepsRequested;
   private int expectedIndex = 0;
   private RobotSide lastSide = null;

   public WalkToGoalWithPlanningBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel, YoDouble yoTime)
   {
      super(outgoingCommunicationBridge);
      DEBUG.set(true);
      this.yoTime = yoTime;

      isDone = new YoBoolean("isDone", registry);
      hasInputBeenSet = new YoBoolean("hasInputsBeenSet", registry);

      this.fullRobotModel = fullRobotModel;

      this.attachNetworkListeningQueue(inputListeningQueue, WalkToGoalBehaviorPacket.class);
      this.attachNetworkListeningQueue(plannedPathListeningQueue, FootstepPathPlanPacket.class);
      this.attachNetworkListeningQueue(footstepStatusQueue, FootstepStatusMessage.class);
   }

   @Override
   public void doControl()
   {
      if (!isDone.getBooleanValue())
      {
         checkForNewInputs();
      }
      if (!hasInputBeenSet())
      {
         return;
      }
      if (atGoal())
      {
         debugPrintln("At goal, plan complete");
         hasInputBeenSet.set(false);
         executePlan.set(false);
         return;
      }
      if (checkForNewPlan())
      {
         return;
      }
      if (checkForStepCompleted())
      {
         return;
      }

      if (executePlan.getBooleanValue() && hasNewPlan.getBooleanValue() && (!stepCompleted.getBooleanValue() || allStepsCompleted.getBooleanValue()))
      {
         if (planValid(currentPlan) && (currentPlan.getFootstepUnknown().get(1) == 0 || executeUnknownFirstStep.getBooleanValue()))
         {
            processNextStep();
            return;
         }
      }

      if (currentPlan != null && currentPlan.getFootstepUnknown() != null && (currentPlan.getFootstepUnknown().isEmpty() || currentPlan.getFootstepUnknown().get(1) == 1)
            && requestQuickSearch.getBooleanValue())
      {
         //no plan or next step unknown, need a new plan fast
         debugPrintln("Quick Search Requested");
         requestFootstepPlan();
         requestQuickSearch.set(false);

      }
   }

   private boolean checkForNewPlan()
   {
      //return true if new packet, else return false.
      FootstepPathPlanPacket newestPacket = plannedPathListeningQueue.poll();
      if (newestPacket != null)
      {
         System.out.println("New plan received. Checking validity...");
         if (planValid(newestPacket))
         {
            currentPlan = newestPacket;
            debugPrintln("Valid plan, new plan is:");
            debugPrintln(currentPlan.getPathPlan().toString());
            hasNewPlan.set(true);
            //stop current steps
            visualizePlan(currentPlan);
         }
         else
         {
            //				visualizePlan(newestPacket);
            System.out.println("Plan is not Valid!");
         }
         return true;
      }
      return false;
   }

   private void visualizePlan(FootstepPathPlanPacket plan)
   {
      if (plan.getPathPlan() == null || plan.getPathPlan().isEmpty())
         return;
      int size = plan.getPathPlan().size();
      SnapFootstepPacket planVisualizationPacket = new SnapFootstepPacket();

      for (int i = 0; i < size; i++)
      {
         planVisualizationPacket.getFootstepData().add().set(plan.getPathPlan().get(i));
         planVisualizationPacket.getFootstepOrder().add(i);
         planVisualizationPacket.getFlag().add((byte) (plan.getFootstepUnknown().get(i) == 1 ? 0 : 2));
      }
      planVisualizationPacket.setDestination(PacketDestination.NETWORK_PROCESSOR.ordinal());
      sendPacket(planVisualizationPacket);
   }

   private boolean planValid(FootstepPathPlanPacket plan)
   {
      if (plan == null)
         return false;
      if (!plan.getGoalsValid())
         return false;
      for (int i = 0; i < plan.getOriginalGoals().size(); i++)
      {
         if (!approximatelyEqual(plan.getOriginalGoals().get(i), goalFootsteps.get(i)))
            return false;
      }
      while (plan.getPathPlan().size() > 0 && !approximatelyEqual(plan.getPathPlan().get(0), predictedLocation))
      {
         plan.getPathPlan().remove(0);
         plan.getFootstepUnknown().removeAt(0);
      }
      if (plan.getPathPlan().size() < 2)
         return false;
      if (approximatelyEqual(plan.getPathPlan().get(1), currentLocation))
         return false;
      waitingForValidPlan.set(false);
      return true;
   }

   private boolean checkForStepCompleted()
   {
      //return true if there was a new packet, otherwise return false.
      FootstepStatusMessage newestPacket = footstepStatusQueue.poll();
      if (newestPacket != null)
      {
         //TODO: update current location and predicted location from the feedback
         if (newestPacket.getFootstepStatus() == FootstepStatus.STARTED.toByte())
         {
            stepCompleted.set(false);
            debugPrintln("Number of requested steps: " + stepsRequested.size());
            debugPrintln("footstep index: " + newestPacket.getFootstepIndex());
            debugPrintln("expected index: " + expectedIndex);
            predictedLocation = stepsRequested.get(expectedIndex);
            debugPrintln("Predicted now at " + predictedLocation.toString());
            sendUpdateStart(predictedLocation);
            expectedIndex++;
         }
         else if (newestPacket.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
         {
            stepCompleted.set(true);
            currentLocation = predictedLocation;
            RobotSide robotSide = RobotSide.fromByte(newestPacket.getRobotSide());
            FootstepDataMessage actualFootstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, newestPacket.getActualFootPositionInWorld(), newestPacket.getActualFootOrientationInWorld());
            lastSide = robotSide;

            debugPrintln("Step Completed, expected location is: " + currentLocation.toString());
            debugPrintln("Step Completed, actual location is: " + actualFootstep.toString());
            if (newestPacket.getFootstepIndex() == stepsRequested.size() - 1)
            {
               debugPrintln("All steps complete");
               allStepsCompleted.set(true);
               requestQuickSearch.set(true);
            }
         }
         return true;
      }
      return false;
   }

   private void processNextStep()
   {
      if (!planValid(currentPlan))
      {
         debugPrintln("current plan is invalid, waiting for new plan");
         if (!waitingForValidPlan.getBooleanValue())
         {
            sendUpdateStart(predictedLocation);
            waitingForValidPlan.set(true);
         }
      }
      else
      {
         takeStep();
         hasNewPlan.set(false);
      }
   }

   private void takeStep()
   {
      //remove current location from plan, element 1 is next step
      currentPlan.getPathPlan().remove(0);
      currentPlan.getFootstepUnknown().removeAt(0);
      //element 1 is now element 0
      if (currentPlan.getFootstepUnknown().get(0) == 1 && !executeUnknownFirstStep.getBooleanValue())
         return;

      sendStepsToController();
      stepCompleted.set(false);

   }

   private boolean atGoal()
   {
      if (currentLocation == null)
         return false;
      for (FootstepDataMessage goal : goalFootsteps)
      {
         if (approximatelyEqual(currentLocation, goal))
         {
            hasInputBeenSet.set(false);
            return true;
         }
      }
      return false;
   }

   private boolean approximatelyEqual(FootstepDataMessage currentLocation, FootstepDataMessage checkAgainst)
   {
      if (currentLocation == null)
         return false;
      double xDiff = currentLocation.getLocation().getX() - checkAgainst.getLocation().getX();
      double yDiff = currentLocation.getLocation().getY() - checkAgainst.getLocation().getY();
      if (currentLocation.getRobotSide() != checkAgainst.getRobotSide())
         return false;
      if (Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)) > 0.05)
         return false;
      if (Math.abs(currentLocation.getOrientation().getYaw() - checkAgainst.getOrientation().getYaw()) > Math.PI / 16)
         return false;
      return true;
   }

   private void requestFootstepPlan()
   {
      FootstepPlanRequestPacket footstepPlanRequestPacket = HumanoidMessageTools.createFootstepPlanRequestPacket(FootstepPlanRequestType.START_SEARCH, startFootstep, startYaw, goalFootsteps, 10);
      communicationBridge.sendPacket(footstepPlanRequestPacket);
      waitingForValidPlan.set(true);
   }

   private void requestSearchStop()
   {
      FootstepPlanRequestPacket stopSearchRequestPacket = HumanoidMessageTools.createFootstepPlanRequestPacket(FootstepPlanRequestType.STOP_SEARCH, new FootstepDataMessage(), 0.0, null);
      communicationBridge.sendPacket(stopSearchRequestPacket);
      waitingForValidPlan.set(false);
   }

   private void sendUpdateStart(FootstepDataMessage updatedLocation)
   {
      if (updatedLocation.getOrientation().epsilonEquals(new Quaternion(), .003))
         return;
      FootstepPlanRequestPacket updateStartPacket = HumanoidMessageTools.createFootstepPlanRequestPacket(FootstepPlanRequestType.UPDATE_START, updatedLocation, updatedLocation.getOrientation().getYaw(), null, 10);
      communicationBridge.sendPacket(updateStartPacket);
   }

   private void sendStepsToController()
   {
      int size = currentPlan.getFootstepUnknown().size();
      FootstepDataListMessage outgoingFootsteps = new FootstepDataListMessage();
      for (int i = 0; i < size; i++)
      {
         if (executeUnknownFirstStep.getBooleanValue() && i == 0)
         {
            outgoingFootsteps.getFootstepDataList().add().set(currentPlan.getPathPlan().get(i));
            executeUnknownFirstStep.set(false);
         }
         else if (currentPlan.getFootstepUnknown().get(i) == 0)
         {
            outgoingFootsteps.getFootstepDataList().add().set(currentPlan.getPathPlan().get(i));
         }
         else
         {
            break;
         }
         debugPrintln("Step Added To Send: " + outgoingFootsteps.getFootstepDataList().get(i));
         if (outgoingFootsteps.getFootstepDataList().get(i).getPredictedContactPoints2d() != null
               && outgoingFootsteps.getFootstepDataList().get(i).getPredictedContactPoints2d().isEmpty())
         {
            debugPrintln("Support Points are: " + outgoingFootsteps.getFootstepDataList().get(i).getPredictedContactPoints2d().toString());
            throw new RuntimeException("attempting to send footstep with empty list of support points");
         }
      }
      stepsRequested = outgoingFootsteps.getFootstepDataList();
      debugPrintln(stepsRequested.size() + " steps sent to controller");
      debugPrintln(stepsRequested.toString());
      outgoingFootsteps.setDestination(PacketDestination.CONTROLLER.ordinal());
      allStepsCompleted.set(false);
      sendPacketToController(outgoingFootsteps);
      expectedIndex = 0;
   }

   private void checkForNewInputs()
   {
      WalkToGoalBehaviorPacket newestPacket = inputListeningQueue.poll();
      if (newestPacket != null)
      {
         if (newestPacket.getWalkToGoalAction() == WalkToGoalAction.FIND_PATH.toByte())
         {
            set(newestPacket.getXGoal(), newestPacket.getYGoal(), newestPacket.getThetaGoal(), RobotSide.fromByte(newestPacket.getGoalRobotSide()));
            requestFootstepPlan();
            hasInputBeenSet.set(true);
            debugPrintln("Requesting path");
         }
         else if (newestPacket.getWalkToGoalAction() == WalkToGoalAction.EXECUTE.toByte())
         {
            debugPrintln("Executing path");
            sendPacketToController(HumanoidMessageTools.createPauseWalkingMessage(false));
            executePlan.set(true);
         }
         else if (newestPacket.getWalkToGoalAction() == WalkToGoalAction.EXECUTE_UNKNOWN.toByte())
         {
            executeUnknownFirstStep.set(true);
            debugPrintln("First step now allowed to be unknown");
         }
         else if (newestPacket.getWalkToGoalAction() == WalkToGoalAction.STOP.toByte())
         {
            executePlan.set(false);
            sendPacketToController(HumanoidMessageTools.createPauseWalkingMessage(true));
            debugPrintln("Stopping execution");
         }
      }
   }

   public void set(double xGoal, double yGoal, double thetaGoal, RobotSide goalSide)
   {
      RobotSide startSide;
      if (lastSide == null)
      {
         startSide = RobotSide.LEFT;
      }
      else
      {
         startSide = lastSide;
      }
      Vector3D startTranslation = new Vector3D();
      RotationMatrix startRotation = new RotationMatrix();
      fullRobotModel.getFoot(startSide).getBodyFixedFrame().getTransformToDesiredFrame(worldFrame).getTranslation(startTranslation);
      fullRobotModel.getFoot(startSide).getBodyFixedFrame().getTransformToDesiredFrame(worldFrame).getRotation(startRotation);
      startYaw = startRotation.getYaw();
      Quaternion startOrientation = new Quaternion();
      startOrientation.set(startRotation);
      startFootstep = HumanoidMessageTools.createFootstepDataMessage(startSide, new Point3D(startTranslation), startOrientation);
      currentLocation = new FootstepDataMessage(startFootstep);
      predictedLocation = currentLocation;
      stepCompleted.set(true);

      double robotYOffset = 0.1;
      double xOffset = -1 * robotYOffset * Math.sin(thetaGoal);
      double yOffset = robotYOffset * Math.cos(thetaGoal);

      goalFootsteps.clear();
      //set left foot goal
      /*
       * Quat4d leftGoalOrientation = new Quat4d();
       * RotationFunctions.setQuaternionBasedOnYawPitchRoll(leftGoalOrientation, thetaGoal, 0,0);
       * goalFootsteps.add(new FootstepData(RobotSide.LEFT, new Point3D(xGoal + xOffset, yGoal +
       * yOffset, 0), leftGoalOrientation));
       */

      Quaternion rightGoalOrientation = new Quaternion();
      rightGoalOrientation.setYawPitchRoll(thetaGoal, 0, 0);
      goalFootsteps.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(xGoal - xOffset, yGoal - yOffset, 0), rightGoalOrientation));

      hasInputBeenSet.set(true);
   }

   @Override
   public void onBehaviorEntered()
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
   public void onBehaviorAborted()
   {
      requestSearchStop();
      sendPacketToController(HumanoidMessageTools.createPauseWalkingMessage(true));
      waitingForValidPlan.set(false);
      isAborted.set(true);
   }

   @Override
   public void onBehaviorPaused()
   {

      isPaused.set(true);
      sendPacketToController(HumanoidMessageTools.createPauseWalkingMessage(true));
   }

   @Override
   public void onBehaviorResumed()
   {
      isPaused.set(false);
      sendPacketToController(HumanoidMessageTools.createPauseWalkingMessage(false));
   }

   @Override
   public boolean isDone()
   {
      return atGoal();
   }

   @Override
   public void onBehaviorExited()
   {
      isPaused.set(false);
      isAborted.set(false);
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

   public void debugPrintln(String string)
   {
      if (DEBUG.getBooleanValue())
         System.out.println(string);
   }
}
