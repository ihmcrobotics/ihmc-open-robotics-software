package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPathPlanPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus.Status;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * WalkToLocation with path-planning algorithm to avoid keepout regions
 *
 * @author Alex Graber-Tilton
 *
 */
public class WalkToGoalBehavior extends AbstractBehavior
{

   private final BooleanYoVariable DEBUG = new BooleanYoVariable("DEBUG", registry);
   private final DoubleYoVariable yoTime;
   private double searchStartTime = 0;

   private final ConcurrentListeningQueue<WalkToGoalBehaviorPacket> inputListeningQueue = new ConcurrentListeningQueue<WalkToGoalBehaviorPacket>(20);
   private final ConcurrentListeningQueue<FootstepPathPlanPacket> plannedPathListeningQueue = new ConcurrentListeningQueue<FootstepPathPlanPacket>(20);
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(100);
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable hasInputBeenSet;
   private final FullHumanoidRobotModel fullRobotModel;

   private FootstepDataMessage startFootstep;
   private ArrayList<FootstepDataMessage> goalFootsteps = new ArrayList<FootstepDataMessage>();
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

   private FootstepDataMessage currentLocation;
   private FootstepDataMessage predictedLocation;
   private FootstepPathPlanPacket currentPlan;
   private List<FootstepDataMessage> stepsRequested;
   private double ankleHeight = 0;
   private int expectedIndex = 0;
   private RobotSide lastSide = null;

   public WalkToGoalBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel, DoubleYoVariable yoTime,
                             double ankleHeight)
   {
      super(outgoingCommunicationBridge);
      DEBUG.set(true);
      this.yoTime = yoTime;
      this.ankleHeight = ankleHeight;

      isDone = new BooleanYoVariable("isDone", registry);
      hasInputBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);

      this.fullRobotModel = fullRobotModel;

      this.attachNetworkListeningQueue(inputListeningQueue, WalkToGoalBehaviorPacket.class);
      this.attachNetworkListeningQueue(plannedPathListeningQueue, FootstepPathPlanPacket.class);
      this.attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
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
         if (planValid(currentPlan) && (!currentPlan.footstepUnknown.get(1) || executeUnknownFirstStep.getBooleanValue()))
         {
            processNextStep();
            return;
         }
      }

      if (currentPlan != null && currentPlan.footstepUnknown != null && (currentPlan.footstepUnknown.isEmpty() || currentPlan.footstepUnknown.get(1))
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
            debugPrintln(currentPlan.pathPlan.toString());
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
      if (plan.pathPlan == null || plan.pathPlan.size() == 0)
         return;
      int size = plan.pathPlan.size();
      SnapFootstepPacket planVisualizationPacket = new SnapFootstepPacket();
      planVisualizationPacket.footstepData = new ArrayList<FootstepDataMessage>();
      planVisualizationPacket.footstepOrder = new int[size];
      planVisualizationPacket.flag = new byte[size];

      for (int i = 0; i < size; i++)
      {
         planVisualizationPacket.footstepData.add(adjustFootstepForAnkleHeight(plan.pathPlan.get(i)));
         planVisualizationPacket.footstepOrder[i] = i;
         planVisualizationPacket.flag[i] = (byte) (plan.footstepUnknown.get(i) ? 0 : 2);
      }
      planVisualizationPacket.setDestination(PacketDestination.NETWORK_PROCESSOR);
      sendPacket(planVisualizationPacket);
   }

   private boolean planValid(FootstepPathPlanPacket plan)
   {
      if (plan == null)
         return false;
      if (!plan.goalsValid)
         return false;
      for (int i = 0; i < plan.originalGoals.size(); i++)
      {
         if (!approximatelyEqual(plan.originalGoals.get(i), goalFootsteps.get(i)))
            return false;
      }
      while (plan.pathPlan.size() > 0 && !approximatelyEqual(plan.pathPlan.get(0), predictedLocation))
      {
         plan.pathPlan.remove(0);
         plan.footstepUnknown.remove(0);
      }
      if (plan.pathPlan.size() < 2)
         return false;
      if (approximatelyEqual(plan.pathPlan.get(1), currentLocation))
         return false;
      waitingForValidPlan.set(false);
      return true;
   }

   private boolean checkForStepCompleted()
   {
      //return true if there was a new packet, otherwise return false.
      FootstepStatus newestPacket = footstepStatusQueue.poll();
      if (newestPacket != null)
      {
         //TODO: update current location and predicted location from the feedback
         if (newestPacket.status == Status.STARTED)
         {
            stepCompleted.set(false);
            debugPrintln("Number of requested steps: " + stepsRequested.size());
            debugPrintln("footstep index: " + newestPacket.footstepIndex);
            debugPrintln("expected index: " + expectedIndex);
            predictedLocation = stepsRequested.get(expectedIndex);
            debugPrintln("Predicted now at " + predictedLocation.toString());
            sendUpdateStart(predictedLocation);
            expectedIndex++;
         }
         else if (newestPacket.status == Status.COMPLETED)
         {
            stepCompleted.set(true);
            currentLocation = predictedLocation;
            FootstepDataMessage actualFootstep = new FootstepDataMessage(newestPacket.getRobotSide(), newestPacket.getActualFootPositionInWorld(),
                                                                         newestPacket.getActualFootOrientationInWorld());
            lastSide = newestPacket.getRobotSide();

            debugPrintln("Step Completed, expected location is: " + currentLocation.toString());
            debugPrintln("Step Completed, actual location is: " + actualFootstep.toString());
            if (newestPacket.footstepIndex == stepsRequested.size() - 1)
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
      currentPlan.pathPlan.remove(0);
      currentPlan.footstepUnknown.remove(0);
      //element 1 is now element 0
      if (currentPlan.footstepUnknown.get(0) && !executeUnknownFirstStep.getBooleanValue())
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
      double xDiff = currentLocation.location.getX() - checkAgainst.location.getX();
      double yDiff = currentLocation.location.getY() - checkAgainst.location.getY();
      if (currentLocation.robotSide != checkAgainst.robotSide)
         return false;
      if (Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)) > 0.05)
         return false;
      if (Math.abs(currentLocation.orientation.getYaw() - checkAgainst.orientation.getYaw()) > Math.PI / 16)
         return false;
      return true;
   }

   private void requestFootstepPlan()
   {
      FootstepPlanRequestPacket footstepPlanRequestPacket = new FootstepPlanRequestPacket(FootstepPlanRequestPacket.RequestType.START_SEARCH, startFootstep,
                                                                                          startYaw, goalFootsteps, 10);
      communicationBridge.sendPacket(footstepPlanRequestPacket);
      waitingForValidPlan.set(true);
   }

   private void requestSearchStop()
   {
      FootstepPlanRequestPacket stopSearchRequestPacket = new FootstepPlanRequestPacket(FootstepPlanRequestPacket.RequestType.STOP_SEARCH,
                                                                                        new FootstepDataMessage(), 0.0, null);
      communicationBridge.sendPacket(stopSearchRequestPacket);
      waitingForValidPlan.set(false);
   }

   private void sendUpdateStart(FootstepDataMessage updatedLocation)
   {
      if (updatedLocation.orientation.epsilonEquals(new Quaternion(), .003))
         return;
      FootstepPlanRequestPacket updateStartPacket = new FootstepPlanRequestPacket(FootstepPlanRequestPacket.RequestType.UPDATE_START, updatedLocation,
                                                                                  updatedLocation.orientation.getYaw(), null, 10);
      communicationBridge.sendPacket(updateStartPacket);
   }

   private void sendStepsToController()
   {
      int size = currentPlan.footstepUnknown.size();
      FootstepDataListMessage outgoingFootsteps = new FootstepDataListMessage();
      for (int i = 0; i < size; i++)
      {
         if (executeUnknownFirstStep.getBooleanValue() && i == 0)
         {
            outgoingFootsteps.footstepDataList.add(adjustFootstepForAnkleHeight(currentPlan.pathPlan.get(i)));
            executeUnknownFirstStep.set(false);
         }
         else if (!currentPlan.footstepUnknown.get(i))
         {
            outgoingFootsteps.footstepDataList.add(adjustFootstepForAnkleHeight(currentPlan.pathPlan.get(i)));
         }
         else
         {
            break;
         }
         debugPrintln("Step Added To Send: " + outgoingFootsteps.footstepDataList.get(i));
         if (outgoingFootsteps.footstepDataList.get(i).predictedContactPoints != null
               && outgoingFootsteps.footstepDataList.get(i).predictedContactPoints.isEmpty())
         {
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

   private FootstepDataMessage adjustFootstepForAnkleHeight(FootstepDataMessage footstep)
   {
      FootstepDataMessage copy = new FootstepDataMessage(footstep);
      Point3D ankleOffset = new Point3D(0, 0, ankleHeight);
      RigidBodyTransform footTransform = new RigidBodyTransform();
      footTransform.setRotationAndZeroTranslation(copy.getOrientation());
      footTransform.transform(ankleOffset);
      copy.getLocation().add(ankleOffset);
      return copy;
   }

   private void checkForNewInputs()
   {
      WalkToGoalBehaviorPacket newestPacket = inputListeningQueue.poll();
      if (newestPacket != null)
      {
         if (newestPacket.action == WalkToGoalBehaviorPacket.WalkToGoalAction.FIND_PATH)
         {
            set(newestPacket.getGoalPosition()[0], newestPacket.getGoalPosition()[1], newestPacket.getGoalPosition()[2], newestPacket.getGoalSide());
            requestFootstepPlan();
            hasInputBeenSet.set(true);
            debugPrintln("Requesting path");
         }
         else if (newestPacket.action == WalkToGoalBehaviorPacket.WalkToGoalAction.EXECUTE)
         {
            debugPrintln("Executing path");
            sendPacketToController(new PauseWalkingMessage(false));
            executePlan.set(true);
         }
         else if (newestPacket.action == WalkToGoalBehaviorPacket.WalkToGoalAction.EXECUTE_UNKNOWN)
         {
            executeUnknownFirstStep.set(true);
            debugPrintln("First step now allowed to be unknown");
         }
         else if (newestPacket.action == WalkToGoalBehaviorPacket.WalkToGoalAction.STOP)
         {
            executePlan.set(false);
            sendPacketToController(new PauseWalkingMessage(true));
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
      startFootstep = new FootstepDataMessage(startSide, new Point3D(startTranslation), startOrientation);
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
      goalFootsteps.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(xGoal - xOffset, yGoal - yOffset, 0), rightGoalOrientation));

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
      sendPacketToController(new PauseWalkingMessage(true));
      waitingForValidPlan.set(false);
      isAborted.set(true);
   }

   @Override
   public void onBehaviorPaused()
   {

      isPaused.set(true);
      sendPacketToController(new PauseWalkingMessage(true));
   }

   @Override
   public void onBehaviorResumed()
   {
      isPaused.set(false);
      sendPacketToController(new PauseWalkingMessage(false));
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
