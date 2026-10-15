package us.ihmc.gr00t;

import controller_msgs.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.robotics.robotSide.RobotSide;

import java.nio.DoubleBuffer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * Reusable command lifecycle, preview, safety gates, and execution for a humanoid GR00T embodiment.
 * <p>
 * The task is selected entirely by the operator's language command. No object, phase, or activity
 * such as a lever or tennis ball is encoded here.
 */
public class Gr00tHumanoidTask extends Gr00tHumanoidDiagnostics implements Gr00tTask
{
   public record PreviewState(Pose3D rootPose, double[] jointPositions, long sequence)
   {
      public PreviewState
      {
         rootPose = new Pose3D(rootPose);
         jointPositions = jointPositions.clone();
      }

      @Override
      public Pose3D rootPose()
      {
         return new Pose3D(rootPose);
      }

      @Override
      public double[] jointPositions()
      {
         return jointPositions.clone();
      }
   }

   private final Gr00tModelConfiguration modelConfiguration;
   private final Gr00tHumanoidConfiguration humanoidConfiguration;
   private final Gr00tHandController hands;
   private final Gr00tHumanoidPlanExecutor plans;
   private final BiFunction<Gr00tClient, Gr00tHandController, Gr00tObservationSource> observationSourceFactory;
   private final ROS2Publisher<StopAllTrajectoryMessage> stopAllTrajectoryPublisher;
   private volatile Consumer<String> statusConsumer = status -> { };
   private volatile String command = System.getProperty("gr00t.prompt", "").strip();
   private volatile String currentStatus = "Not connected to GR00T bridge";
   private volatile long commandGeneration;
   private volatile boolean running;
   private volatile boolean stopAllTrajectoriesRequested;
   private volatile boolean inferenceResetRequested;

   protected Gr00tHumanoidTask(AsyncROS2Node ros2Node,
                               DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               String registryName,
                               Gr00tModelConfiguration modelConfiguration,
                               Gr00tHumanoidConfiguration humanoidConfiguration,
                               Gr00tActionDecoder<Gr00tHumanoidAction> actionDecoder,
                               String[] handTargetNames,
                               Function<Consumer<String>, Gr00tHandController> handControllerFactory,
                               BiFunction<Gr00tClient, Gr00tHandController, Gr00tObservationSource> observationSourceFactory)
   {
      super(registryName, handTargetNames);
      this.modelConfiguration = modelConfiguration;
      this.humanoidConfiguration = humanoidConfiguration;
      this.observationSourceFactory = observationSourceFactory;
      hands = handControllerFactory.apply(this::updateStatus);
      plans = new Gr00tHumanoidPlanExecutor(ros2Node,
                                            robotModel,
                                            syncedRobot,
                                            humanoidConfiguration,
                                            actionDecoder,
                                            this,
                                            hands,
                                            this::updateStatus);
      stopAllTrajectoryPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(StopAllTrajectoryMessage.class,
                                                                                           robotModel.getSimpleRobotName()));
   }

   @Override
   public Gr00tModelConfiguration getModelConfiguration()
   {
      return modelConfiguration;
   }

   public Gr00tHumanoidConfiguration getHumanoidConfiguration()
   {
      return humanoidConfiguration;
   }

   @Override
   public Gr00tObservationSource createObservationSource(Gr00tClient client)
   {
      return observationSourceFactory.apply(client, hands);
   }

   @Override
   public void setStatusConsumer(Consumer<String> statusConsumer)
   {
      this.statusConsumer = statusConsumer == null ? status -> { } : statusConsumer;
   }

   @Override
   public void observeStatus(String status)
   {
      currentStatus = status;
   }

   @Override
   public void updateBeforeInference()
   {
      hands.update();
      if (stopAllTrajectoriesRequested)
      {
         stopAllTrajectoriesRequested = false;
         stopAllTrajectoryPublisher.publish(new StopAllTrajectoryMessage());
         updateStatus("STOP ALL TRAJECTORIES sent");
      }

      plans.updateBeforeExecution(running);
      Gr00tHumanoidPlanExecutor.Completion completion = plans.pollCompletion();
      if (completion != Gr00tHumanoidPlanExecutor.Completion.NONE)
      {
         plans.clearActiveChunk();
         plans.setPlanStatus("Action chunk complete; ready to continue command: " + command);
      }

      if (!running && !currentStatus.startsWith("STOP"))
         updateStatus("Not running");
   }

   @Override
   public void updateAfterInference()
   {
      plans.updatePreview();
   }

   @Override
   public boolean isRunning()
   {
      return running;
   }

   @Override
   public boolean shouldRequestInference()
   {
      return running && !command.isBlank() && !plans.hasActivePlan();
   }

   @Override
   public Request getRequest()
   {
      return new Request(command, commandGeneration);
   }

   @Override
   public boolean accepts(Request request)
   {
      return request != null && request.generation() == commandGeneration && request.prompt().equals(command);
   }

   @Override
   public boolean consumeInferenceResetRequested()
   {
      boolean requested = inferenceResetRequested;
      inferenceResetRequested = false;
      return requested;
   }

   @Override
   public void recordActionsReceived(int count)
   {
      plans.recordActionsReceived(count);
   }

   @Override
   public void discardAcceptedActionChunk()
   {
      plans.discardAcceptedActionChunk();
   }

   @Override
   public void processActionChunk(DoubleBuffer actionChunk, int realActionCount)
   {
      plans.processActionChunk(actionChunk, realActionCount);
   }

   public String getCommand()
   {
      return command;
   }

   /** Changes tasks without changing Java code; any response for the old command becomes stale. */
   public void setCommand(String command)
   {
      String newCommand = command == null ? "" : command.strip();
      if (newCommand.equals(this.command))
         return;
      if (plans.isControlRobot())
         setControlRobot(false);
      this.command = newCommand;
      commandGeneration++;
      inferenceResetRequested = true;
      plans.clearActiveChunk();
      plans.resetInitialTransit();
      plans.setPlanStatus(newCommand.isBlank() ? "Enter a command before starting inference"
                                               : "Waiting for policy actions for: " + newCommand);
   }

   public void setRunning(boolean running)
   {
      if (running && command.isBlank())
      {
         updateStatus("Enter a command before starting inference");
         this.running = false;
         return;
      }
      if (running && !this.running)
      {
         commandGeneration++;
         inferenceResetRequested = true;
         plans.resetForStart();
         plans.setPlanStatus("Waiting for policy actions for: " + command);
      }
      else if (!running && this.running && plans.isControlRobot())
      {
         setControlRobot(false);
      }
      this.running = running;
   }

   public PreviewState getPendingPreview()
   {
      return plans.getPendingPreview();
   }

   public String getPlanStatus()
   {
      return plans.getPlanStatus();
   }

   public Pose3D getAcceptedWristTarget()
   {
      return plans.getAcceptedWristTarget();
   }

   public double getPlanDuration()
   {
      return plans.getPlanDuration();
   }

   public double getWorstIKQuality()
   {
      return plans.getWorstIKQuality();
   }

   public boolean isAllowPoorIKForTesting()
   {
      return plans.isAllowPoorIKForTesting();
   }

   public void setAllowPoorIKForTesting(boolean allowPoorIKForTesting)
   {
      if (allowPoorIKForTesting && plans.isControlRobot())
         setControlRobot(false);
      plans.setAllowPoorIKForTesting(allowPoorIKForTesting);
   }

   public void stopAllRobotTrajectories()
   {
      plans.setControlRobot(false);
      plans.setControlNeck(false);
      hands.setEnabled(false);
      stopAllTrajectoriesRequested = true;
   }

   public void setControlRobot(boolean controlRobot)
   {
      boolean wasControlRobot = plans.isControlRobot();
      plans.setControlRobot(controlRobot);
      if (controlRobot && !wasControlRobot)
         hands.setEnabled(true);
      else if (!controlRobot && wasControlRobot)
         stopAllTrajectoriesRequested = true;
      if (!controlRobot)
         hands.setEnabled(false);
   }

   public boolean isControlRobot()
   {
      return plans.isControlRobot();
   }

   public void setControlHands(boolean controlHands)
   {
      hands.setEnabled(controlHands);
   }

   public boolean isControlHands()
   {
      return hands.isEnabled();
   }

   public void requestCloseGrip()
   {
      hands.requestGrip(humanoidConfiguration.armSide(), true);
   }

   public void requestOpenGrip()
   {
      hands.requestGrip(humanoidConfiguration.armSide(), false);
   }

   public void setControlNeck(boolean controlNeck)
   {
      plans.setControlNeck(controlNeck);
   }

   public boolean isControlNeck()
   {
      return plans.isControlNeck();
   }

   public boolean hasHandState(RobotSide side)
   {
      return hands.hasState(side);
   }

   private void updateStatus(String status)
   {
      currentStatus = status;
      statusConsumer.accept(status);
   }
}
