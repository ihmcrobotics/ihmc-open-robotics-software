package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.ConstrainedWholeBodyPlanningToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningToolboxRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This behavior is to plan whole body trajectory under constrained end effector.
 * The user input is constrained trajectory for both hand. See {@link ConstrainedEndEffectorTrajectory, ConstrainedConfigurationSpace}
 * The output is WholeBodyTrajectoryMessage. See {@link WholeBodyTrajectoryMessage}
 * @author Edward Inho Lee.
 */
public class PlanConstrainedWholeBodyTrajectoryBehavior extends AbstractBehavior
{
   // TODO : should be replaced with simple packet structure. need to add 'configruationspaces, buildorder etc' on kryonet.
   public static ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   private final boolean DEBUG = true;

   private PipeLine pipeLine = new PipeLine();

   private boolean planningSuccess = false;

   private double timeout = 200.0;

   private final SleepBehavior sleepBehavior;

   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;
   
   private SideDependentList<ArrayList<Pose3D>> handTrajectories = new SideDependentList<>();

   private ConcurrentListeningQueue<ConstrainedWholeBodyPlanningToolboxOutputStatus> cwbtoolboxOutputStatusQueue = new ConcurrentListeningQueue<ConstrainedWholeBodyPlanningToolboxOutputStatus>(20);
   private FullHumanoidRobotModel fullRobotModel;

   private ConstrainedWholeBodyPlanningToolboxOutputStatus cwbtoolboxOutputStatus;
   
   private final ConstrainedWholeBodyPlanningToolboxOutputConverter outputConverter;
   
   private long startTime; // computing time measure.
   
   private static int numberOfFindInitialGuess = 320;
   private static int numberOfExpanding = 1000;
   private static int numberOfEndEffectorWayPoints = 10;

   public PlanConstrainedWholeBodyTrajectoryBehavior(String namePrefix, FullHumanoidRobotModelFactory fullRobotModelFactory, CommunicationBridgeInterface communicationBridge, FullHumanoidRobotModel fullRobotModel,
                                                     YoDouble yoTime)
   {
      super(namePrefix, communicationBridge);

      this.attachNetworkListeningQueue(cwbtoolboxOutputStatusQueue, ConstrainedWholeBodyPlanningToolboxOutputStatus.class);

      this.sleepBehavior = new SleepBehavior(communicationBridge, yoTime);

      this.fullRobotModel = fullRobotModel;
      
      this.outputConverter = new ConstrainedWholeBodyPlanningToolboxOutputConverter(fullRobotModelFactory);
   }
   
   public void setInputs(FullHumanoidRobotModel fullRobotModel)
   {
      
   }

   public void setInputs(ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory, FullHumanoidRobotModel fullRobotModel)
   {
      PlanConstrainedWholeBodyTrajectoryBehavior.constrainedEndEffectorTrajectory = constrainedEndEffectorTrajectory;
      this.fullRobotModel = fullRobotModel;
   }
   
   public void setNumberOfFindInitialGuess(int value)
   {
      numberOfFindInitialGuess = value;
   }
   
   public void setNumberOfExpanding(int value)
   {
      numberOfExpanding = value;
   }
   
   public void setNumberOfEndEffectorWayPoints(int value)
   {
      numberOfEndEffectorWayPoints = value;
   }

   public ConstrainedWholeBodyPlanningToolboxOutputStatus getConstrainedWholeBodyPlanningToolboxOutputStatus()
   {
      return cwbtoolboxOutputStatus;
   }
   
   public ArrayList<Pose3D> getHandTrajectories(RobotSide robotSide)
   {
      return handTrajectories.get(robotSide);
   }

   public WholeBodyTrajectoryMessage getWholebodyTrajectoryMessage()
   {
      return wholebodyTrajectoryMessage;
   }

   private void setupPipeline()
   {
      pipeLine.clearAll();

      BehaviorAction wakeup = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               TextToSpeechPacket p1 = new TextToSpeechPacket("Telling Planner To Wake Up");
               sendPacket(p1);
            }
            ToolboxStateMessage wakeUp = new ToolboxStateMessage(ToolboxState.WAKE_UP);
            sendPackageToPlanner(wakeUp);
            
            PrintTools.info("initialize Planner");
         }
      };

      BehaviorAction requestPlan = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               TextToSpeechPacket p1 = new TextToSpeechPacket("Requesting Plan");
               sendPacket(p1);
            }
            ConstrainedWholeBodyPlanningToolboxRequestPacket request = new ConstrainedWholeBodyPlanningToolboxRequestPacket();
            
            request.setNumberOfFindInitialGuess(numberOfFindInitialGuess);
            request.setNumberOfExpanding(numberOfExpanding);
            request.setInitialRobotConfigration(fullRobotModel);
            request.setNumberOfEndEffectorWayPoints(numberOfEndEffectorWayPoints);

            request.setDestination(PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);

            sendPackageToPlanner(request);
            PrintTools.info("sendPackageToPlanner");
            
            startTime = System.currentTimeMillis();
         }
      };

      BehaviorAction waitForPlan = new BehaviorAction(sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Waiting For Plan");
            sendPacket(p1);

            sleepBehavior.setSleepTime(timeout);
         }

         @Override
         public boolean isDone()
         {
            return super.isDone() || cwbtoolboxOutputStatusQueue.isNewPacketAvailable();
         }
      };

      BehaviorAction processPlan = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (cwbtoolboxOutputStatusQueue.isNewPacketAvailable())
            {
               cwbtoolboxOutputStatus = cwbtoolboxOutputStatusQueue.getLatestPacket();
               if (cwbtoolboxOutputStatus.getPlanningResult() == 4)
               {
                  planningSuccess = true;
                  ConstrainedWholeBodyPlanningToolboxOutputStatus uiPacket = new ConstrainedWholeBodyPlanningToolboxOutputStatus(cwbtoolboxOutputStatus);
                  uiPacket.setDestination(PacketDestination.UI);
                  sendPacketToUI(uiPacket);
                  
                  if(cwbtoolboxOutputStatus.getRobotConfigurations().length == 0)
                     PrintTools.info("something wrong");
                  
                  PrintTools.info("received size of path is "+cwbtoolboxOutputStatus.getRobotConfigurations().length);
                  
                  
                  
                  wholebodyTrajectoryMessage = cwbtoolboxOutputStatus.getWholeBodyTrajectoryMessage();
                  
//                  wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
//                  outputConverter.setConstrainedEndEffectorTrajectory(constrainedEndEffectorTrajectory);
//                  outputConverter.setMessageToCreate(wholebodyTrajectoryMessage);
//                  outputConverter.updateFullRobotModel(cwbtoolboxOutputStatus);
                  
                  // TODO: deactivate toolbox module.
                                                      
                  long stopTime = System.currentTimeMillis();
                  long elapsedTime = stopTime - startTime;
                  System.out.println("===========================================");
                  System.out.println("planning time is " + elapsedTime / 1000.0 + " seconds");
                  System.out.println("===========================================");
               }
               else
               {
                  planningSuccess = false;
               }  
            }
            else
            {
               planningSuccess = false;
            }

            if (planningSuccess)
            {
               if (DEBUG)
               {
                  TextToSpeechPacket p1 = new TextToSpeechPacket("Processing Plan");
                  sendPacket(p1);
               }
            }
            else if (DEBUG)
            {
               TextToSpeechPacket p1 = new TextToSpeechPacket("Plan Failed");
               sendPacket(p1);
            }

            PrintTools.info("set Input process plan");
         }
      };

      pipeLine.requestNewStage();

      pipeLine.submitSingleTaskStage(wakeup);
      pipeLine.submitSingleTaskStage(requestPlan);
      pipeLine.submitSingleTaskStage(waitForPlan);
      pipeLine.submitSingleTaskStage(processPlan);

   }

   private void sendPackageToPlanner(Packet<?> packet)
   {
      packet.setDestination(PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);
      communicationBridge.sendPacket(packet);
   }

   @Override
   public void doControl()
   {
      pipeLine.doControl();

   }

   public boolean planSuccess()
   {
      return planningSuccess;
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
      planningSuccess = false;
      cwbtoolboxOutputStatus = null;
      cwbtoolboxOutputStatusQueue.clear();      

   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

}