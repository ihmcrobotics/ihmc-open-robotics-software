package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkOverTerrainGoalPacket;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationPlannedBehavior.WalkToLocationStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkToLocationPlannedBehavior extends StateMachineBehavior<WalkToLocationStates>
{
   public enum WalkToLocationStates
   {
      WAIT_FOR_GOAL, PLAN_PATH,SLEEP, PLAN_FAILED, WALK_PATH, DONE
   }

   private final HumanoidReferenceFrames referenceFrames;

   private boolean walkSucceded = true;
   

   private final AtomicReference<FramePose3D> currentGoalPose = new AtomicReference<>();
   private final AtomicReference<FramePose3D> newGoalPose = new AtomicReference<>();
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> plannerResult = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();

   private final FootstepListBehavior footstepListBehavior;
   private PlanPathToLocationBehavior planPathToLocationBehavior;
   private final YoDouble yoTime;
   private boolean setupComplete = false;
   private boolean goalLocationChanged = false;
   
   private  FootstepPlannerType footStepPlannerToUse =  FootstepPlannerType.A_STAR;
   private boolean assumeFlatGround = false;


   public WalkToLocationPlannedBehavior(String robotName, Ros2Node ros2Node, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
                                        WalkingControllerParameters walkingControllerParameters, YoDouble yoTime)
   {
      super(robotName, "WalkToLocationBehavior", WalkToLocationStates.class, yoTime, ros2Node);

      this.referenceFrames = referenceFrames;
      this.yoTime = yoTime;

      createSubscribers();

      //setupBehaviors
      planPathToLocationBehavior = new PlanPathToLocationBehavior(robotName, ros2Node, yoTime);
      footstepListBehavior = new FootstepListBehavior(robotName, ros2Node, walkingControllerParameters);


      setupStateMachine();
   }

   @Override
   public void doControl()
   {
      if(newGoalPose.get()!=null)
      {
         setTarget(newGoalPose.getAndSet(null));
      }
      super.doControl();
   }
   
   private void createSubscribers()
   {
      createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlanningToolboxPubGenerator, plannerResult::set);

      createBehaviorInputSubscriber(WalkOverTerrainGoalPacket.class,
                                    (packet) -> newGoalPose.set(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.getPosition(), packet.getOrientation())));
      createSubscriber(PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator, planarRegions::set);
   }

   public void setTarget(FramePose3D targetPoseInWorld)
   {
      publishTextToSpeech("New Goal Target Recieved");
      currentGoalPose.set(targetPoseInWorld);
      goalLocationChanged=true;
   }
   
   public void setFootStepPlanner( FootstepPlannerType footStepPlannerToUse)
   {
      this.footStepPlannerToUse = footStepPlannerToUse;
   }
   
   public void setAssumeFlatGround(boolean assumeFlatGround)
   {
      this.assumeFlatGround = assumeFlatGround;
   }

   @Override
   public void onBehaviorEntered()
   {
      currentGoalPose.set(null);
      walkSucceded = false;
      planPathToLocationBehavior.onBehaviorEntered();
      footstepListBehavior.onBehaviorEntered();
      goalLocationChanged= false;
      super.onBehaviorEntered();
     
   }
   @Override
   protected WalkToLocationStates configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkToLocationStates, BehaviorAction> factory)
   {
      
      

      BehaviorAction waitForGoalToBeSet = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            setupComplete = true;

         }

         @Override
         public boolean isDone(double timeInState)
         {
            // TODO Auto-generated method stub
            return currentGoalPose.get() != null;
         }
      };

      BehaviorAction planFootSteps = new BehaviorAction(planPathToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            goalLocationChanged = false;
            RobotSide initialStanceSide = RobotSide.LEFT;
            RigidBodyTransform soleToWorld = referenceFrames.getSoleFrame(initialStanceSide).getTransformToWorldFrame();
            FramePose3D stanceFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(), soleToWorld);
            if (currentGoalPose.get() == null)
               System.err.println("WalkToLocationPlannedBehavior: goal pose NULL");

            planPathToLocationBehavior.setInputs(currentGoalPose.get(), stanceFootPose, initialStanceSide,footStepPlannerToUse, assumeFlatGround);
            planPathToLocationBehavior.setPlanningTimeout(20);
         }
         
      };

      BehaviorAction sendPlanToController = new BehaviorAction(footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {


            if (planPathToLocationBehavior.getFootStepList() != null)
            {
               footstepListBehavior.set(planPathToLocationBehavior.getFootStepList());
            }
         }


      };
      BehaviorAction doneState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {

            walkSucceded = true;
         }
      };
      BehaviorAction planFailedState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            walkSucceded = false;
            publishTextToSpeech("WalkToLocationPlannedBehavior: Plan Failed");
         }
      };
      
      BehaviorAction PausedState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            
         }
      };

      factory.addState(WalkToLocationStates.WAIT_FOR_GOAL, waitForGoalToBeSet);

      factory.addState(WalkToLocationStates.PLAN_PATH, planFootSteps);

      factory.addStateAndDoneTransition(WalkToLocationStates.WALK_PATH, sendPlanToController, WalkToLocationStates.DONE);
      factory.addStateAndDoneTransition(WalkToLocationStates.PLAN_FAILED, planFailedState, WalkToLocationStates.DONE);
      factory.addState(WalkToLocationStates.DONE, doneState);

      factory.addTransition(WalkToLocationStates.WAIT_FOR_GOAL, WalkToLocationStates.PLAN_PATH, t -> currentGoalPose.get() != null);
      factory.addTransition(WalkToLocationStates.PLAN_PATH, WalkToLocationStates.WALK_PATH, t -> isPlanPathComplete() && hasValidPlanPath());
      factory.addTransition(WalkToLocationStates.PLAN_PATH, WalkToLocationStates.PLAN_PATH, t -> goalLocationChanged);
      factory.addTransition(WalkToLocationStates.WALK_PATH, WalkToLocationStates.PLAN_PATH, t -> goalLocationChanged);

      //factory.addTransition(WalkToLocationStates.PLAN_PATH, WalkToLocationStates.PLAN_FAILED, t -> isPlanPathComplete() && !hasValidPlanPath());
      
      return WalkToLocationStates.WAIT_FOR_GOAL;
   }

   public boolean walkSucceded()
   {
      return walkSucceded;
   }

   @Override
   public void onBehaviorAborted()
   {
      super.onBehaviorAborted();
      footstepListBehavior.onBehaviorAborted();
      isAborted.set(true);
   }

   @Override
   public void onBehaviorPaused()
   {
      super.onBehaviorPaused();
      footstepListBehavior.onBehaviorPaused();
      isPaused.set(true);
   }

   @Override
   public void onBehaviorResumed()
   {
      if(!goalLocationChanged)
      super.onBehaviorResumed();
      else
         getStateMachine().resetCurrentState();
      //i dont think this needs to be called i thiink the line above calls it
      //footstepListBehavior.onBehaviorResumed();
      isPaused.set(false);

   }

   @Override
   public void onBehaviorExited()
   {
      isPaused.set(false);
      isAborted.set(false);
      setupComplete = false;
   }

   private boolean isPlanPathComplete()
   {
      return planPathToLocationBehavior.isDone();
   }

   private boolean hasValidPlanPath()
   {
      return planPathToLocationBehavior.planSuccess();
   }
   
   @Override
   public boolean isDone()
   {
      if(setupComplete)
      return getStateMachine().getCurrentBehaviorKey().equals(WalkToLocationStates.DONE) ||  getStateMachine().getCurrentBehaviorKey().equals(WalkToLocationStates.PLAN_FAILED);
      else
         return false;
   }
}
