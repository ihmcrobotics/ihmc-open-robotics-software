package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.tools.taskExecutor.TaskExecutor;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This behavior is to plan whole body trajectory under constrained end effector. The user input is
 * constrained trajectory for both hand. See {@link ConstrainedEndEffectorTrajectory,
 * ConstrainedConfigurationSpace} The output is WholeBodyTrajectoryMessage. See
 * {@link WholeBodyTrajectoryMessage}
 * 
 * @author Edward Inho Lee.
 */
public class PlanWholeBodyTrajectoryBehavior extends AbstractBehavior
{
   private final boolean DEBUG = true;

   private TaskExecutor taskExecutor = new TaskExecutor();

   private boolean planningSuccess = false;

   private double timeout = 200.0;

   private final SleepBehavior sleepBehavior;

   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;

   private WholeBodyTrajectoryToolboxMessage input = null;
   private ConcurrentListeningQueue<WholeBodyTrajectoryToolboxOutputStatus> wholeBodyTrajectoryToolboxOutputStatusQueue = new ConcurrentListeningQueue<>(20);
   private WholeBodyTrajectoryToolboxOutputStatus wholeBodyTrajectoryToolboxOutputStatus;

   private long startTime; // computing time measure.

   public PlanWholeBodyTrajectoryBehavior(String namePrefix, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                          CommunicationBridgeInterface communicationBridge, FullHumanoidRobotModel fullRobotModel, YoDouble yoTime)
   {
      super(namePrefix, communicationBridge);

      this.attachNetworkListeningQueue(wholeBodyTrajectoryToolboxOutputStatusQueue, WholeBodyTrajectoryToolboxOutputStatus.class);

      this.sleepBehavior = new SleepBehavior(communicationBridge, yoTime);
   }

   public void setInput(WholeBodyTrajectoryToolboxMessage input)
   {
      this.input = input;
   }

   public WholeBodyTrajectoryToolboxOutputStatus getWholeBodyTrajectoryToolboxOutputStatus()
   {
      return wholeBodyTrajectoryToolboxOutputStatus;
   }

   public WholeBodyTrajectoryMessage getWholebodyTrajectoryMessage()
   {
      return wholebodyTrajectoryMessage;
   }

   @SuppressWarnings("rawtypes")
   private void setupPipeline()
   {
      taskExecutor.clear();

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

            sendPackageToPlanner(input);
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
            return super.isDone() || wholeBodyTrajectoryToolboxOutputStatusQueue.isNewPacketAvailable();
         }
      };

      BehaviorAction processPlan = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (wholeBodyTrajectoryToolboxOutputStatusQueue.isNewPacketAvailable())
            {
               wholeBodyTrajectoryToolboxOutputStatus = wholeBodyTrajectoryToolboxOutputStatusQueue.getLatestPacket();
               if (wholeBodyTrajectoryToolboxOutputStatus.getPlanningResult() == 4)
               {
                  planningSuccess = true;
                  WholeBodyTrajectoryToolboxOutputStatus uiPacket = new WholeBodyTrajectoryToolboxOutputStatus(wholeBodyTrajectoryToolboxOutputStatus);
                  uiPacket.setDestination(PacketDestination.UI);
                  sendPacketToUI(uiPacket);

                  if (wholeBodyTrajectoryToolboxOutputStatus.getRobotConfigurations().length == 0)
                     PrintTools.info("something wrong");

                  PrintTools.info("received size of path is " + wholeBodyTrajectoryToolboxOutputStatus.getRobotConfigurations().length);

                  wholebodyTrajectoryMessage = wholeBodyTrajectoryToolboxOutputStatus.getWholeBodyTrajectoryMessage();

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

      taskExecutor.submit(requestPlan);
      taskExecutor.submit(waitForPlan);
      taskExecutor.submit(processPlan);
   }

   private void sendPackageToPlanner(Packet<?> packet)
   {
      packet.setDestination(PacketDestination.WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE);
      communicationBridge.sendPacket(packet);
   }

   @Override
   public void doControl()
   {
      taskExecutor.doControl();

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
      wholeBodyTrajectoryToolboxOutputStatus = null;
      wholeBodyTrajectoryToolboxOutputStatusQueue.clear();

   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
      input = null;
   }

   @Override
   public boolean isDone()
   {
      return taskExecutor.isDone();
   }

}