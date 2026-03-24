package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredTurningVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredVelocityProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.ros2.ROS2HeartbeatMonitor;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeightMapCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class StepGeneratorCommandInputManager implements Updatable
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final CommandInputManager commandInputManager = new CommandInputManager(supportedCommands());

   private boolean isOpen = false;
   private final YoBoolean walk;
   private final YoVector2D desiredVelocity;
   private final YoDouble turningVelocity;
   private final YoDouble swingHeight;
   private final YoBoolean areVelocitiesNormalized;
   private final YoBoolean overrideHeartbeat;
   private HighLevelControllerName currentController;

   private final List<Consumer<HeightMapCommand>> heightMapCommandConsumers = new ArrayList<>();
   private final List<Consumer<ContinuousStepGeneratorParametersCommand>> csgParametersCommandConsumers = new ArrayList<>();
   private final List<Consumer<ControllerWaypointGoalListCommand>> controllerWaypointGoalListCommandConsumer = new ArrayList<>();
   private final List<Consumer<ControllerWaypointGoalCommand>> controllerWaypointGoalCommandConsumer = new ArrayList<>();
   private final List<Consumer<ControllerReleaseGoalCommand>> controllerReleaseGoalCommandConsumers = new ArrayList<>();

   private final AtomicReference<FootstepStatus> latestFootstepStatusReceived = new AtomicReference<>(null);
   private final AtomicReference<FootstepStatus> previousFootstepStatusReceived = new AtomicReference<>(null);
   private final AtomicReference<HeightMapCommand> latestHeightMap = new AtomicReference<>(null);
   private final AtomicReference<WalkingStatus> latestWalkingStatus = new AtomicReference<>(null);
   private final AtomicReference<WalkingStatus> previousWalkingStatus = new AtomicReference<>(null);
   private final AtomicBoolean shouldSubmitNewRegions = new AtomicBoolean(true);
   private final AtomicInteger ticksSinceUpdatingTheEnvironment = new AtomicInteger(0);

   // ROS 2 stuff
   private final ROS2Node ros2Node;
   private final ROS2HeartbeatMonitor heartbeatMonitor;

   public StepGeneratorCommandInputManager()
   {
      String suffix = "StepGeneratorCommandInputManager";
      walk = new YoBoolean("walk_" + suffix, registry);
      desiredVelocity = new YoVector2D("desiredVelocity_" + suffix, registry);
      turningVelocity = new YoDouble("desiredTurningVelocity_" + suffix, registry);
      swingHeight = new YoDouble("desiredSwingHeight_" + suffix, registry);
      areVelocitiesNormalized = new YoBoolean("areVelocitiesNormalized_" + suffix, registry);
      areVelocitiesNormalized.set(true);
      overrideHeartbeat = new YoBoolean("overrideHeartbeat_" + suffix, registry);
      overrideHeartbeat.set(false);

      // by default, command input manager is not enabled, set enabled only if #update is called
      commandInputManager.setEnabled(false);

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase() + "Node");
      heartbeatMonitor = new ROS2HeartbeatMonitor(ros2Node, CSGROS2CommunicationHelper.CSG_HEARTBEAT_TOPIC);
   }

   public void addHeightMapCommandConsumer(Consumer<HeightMapCommand> heightMapCommandConsumer)
   {
      heightMapCommandConsumers.add(heightMapCommandConsumer);
   }

   public void addContinuousStepGeneratorParametersCommandConsumer(Consumer<ContinuousStepGeneratorParametersCommand> csgParametersCommandConsumer)
   {
      csgParametersCommandConsumers.add(csgParametersCommandConsumer);
   }

   public void addControllerWaypointGoalListCommandConsumer(Consumer<ControllerWaypointGoalListCommand> controllerWaypointGoalListCommandConsumer)
   {
      this.controllerWaypointGoalListCommandConsumer.add(controllerWaypointGoalListCommandConsumer);
   }

   public void addControllerWaypointGoalCommandConsumer(Consumer<ControllerWaypointGoalCommand> controllerWaypointGoalCommandConsumer)
   {
      this.controllerWaypointGoalCommandConsumer.add(controllerWaypointGoalCommandConsumer);
   }

   public void addControllerReleaseGoalCommand(Consumer<ControllerReleaseGoalCommand> controllerReleaseGoalCommandConsumer)
   {
      controllerReleaseGoalCommandConsumers.add(controllerReleaseGoalCommandConsumer);
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      return StepGeneratorAPIDefinition.getStepGeneratorSupportedCommands();
   }

   public void setHighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage message)
   {
      currentController = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
   }

   public void setWalkingStatus(WalkingStatusMessage message)
   {
      latestWalkingStatus.set(WalkingStatus.fromByte(message.getWalkingStatus()));
   }

   public void consumeFootstepStatus(FootstepStatusMessage statusMessage)
   {
      FootstepStatus status = FootstepStatus.fromByte(statusMessage.getFootstepStatus());
      if (status == FootstepStatus.COMPLETED)
         notifyFootstepCompleted(RobotSide.fromByte(statusMessage.getRobotSide()));
      else if (status == FootstepStatus.STARTED)
         notifyFootstepStarted();
   }

   public void notifyFootstepCompleted(RobotSide robotSide)
   {
      latestFootstepStatusReceived.set(FootstepStatus.COMPLETED);
   }

   /**
    * Notifies this generator that a footstep has been started, i.e. the foot started swinging.
    * <p>
    * It is used internally to mark the first generated footstep as unmodifiable so it does not change
    * while the swing foot is targeting it.
    * </p>
    */
   public void notifyFootstepStarted()
   {
      latestFootstepStatusReceived.set(FootstepStatus.STARTED);
   }


   @Override
   public void update(double time)
   {
      isOpen = true;
      commandInputManager.setEnabled(isOpen);

      if (!overrideHeartbeat.getValue() && !heartbeatMonitor.isAlive())
      {
         desiredVelocity.setX(0.0);
         desiredVelocity.setY(0.0);
         turningVelocity.set(0.0);
         areVelocitiesNormalized.set(true);
         walk.set(false);
      }
      else if (commandInputManager.isNewCommandAvailable(ContinuousStepGeneratorInputCommand.class))
      {
         ContinuousStepGeneratorInputCommand command = commandInputManager.pollNewestCommand(ContinuousStepGeneratorInputCommand.class);
         desiredVelocity.setX(command.getForwardVelocity());
         desiredVelocity.setY(command.getLateralVelocity());
         turningVelocity.set(command.getTurnVelocity());https://github.com/ihmcrobotics/ihmc-crocoddyl-wrapper/pull/434
         areVelocitiesNormalized.set(command.getAreVelocitiesNormalized());
         walk.set(command.getWalk());
      }
      else if (commandInputManager.isNewCommandAvailable(ControllerWaypointGoalCommand.class))
      {
         ControllerWaypointGoalCommand command = commandInputManager.pollNewestCommand(ControllerWaypointGoalCommand.class);
         for (int i = 0; i < controllerWaypointGoalCommandConsumer.size(); i++)
         {
            controllerWaypointGoalCommandConsumer.get(i).accept(command);
         }
         walk.set(true);
      }
      else if (commandInputManager.isNewCommandAvailable(ControllerWaypointGoalListCommand.class))
      {
         ControllerWaypointGoalListCommand listCommand = commandInputManager.pollNewestCommand(ControllerWaypointGoalListCommand.class);
         for (int w = 0; w < listCommand.getNumberOfWaypoints(); w++)
         {
            ControllerWaypointGoalCommand waypoint = listCommand.getWaypoint(w);
            for (int i = 0; i < controllerWaypointGoalCommandConsumer.size(); i++)
            {
               controllerWaypointGoalCommandConsumer.get(i).accept(waypoint);
            }
         }
         walk.set(true);
      }
      commandInputManager.clearCommands(ContinuousStepGeneratorInputCommand.class);
      commandInputManager.clearCommands(ControllerWaypointGoalCommand.class);
      commandInputManager.clearCommands(ControllerWaypointGoalListCommand.class);

      if (commandInputManager.isNewCommandAvailable(ContinuousStepGeneratorParametersCommand.class))
      {
         ContinuousStepGeneratorParametersCommand command = commandInputManager.pollNewestCommand(ContinuousStepGeneratorParametersCommand.class);
         ContinuousStepGeneratorParameters parameters = command.getParameters();

         swingHeight.set(parameters.getSwingHeight());

         for (int i = 0; i < csgParametersCommandConsumers.size(); i++)
            csgParametersCommandConsumers.get(i).accept(command);
      }
      commandInputManager.clearCommands(ContinuousStepGeneratorParametersCommand.class);

      // update the local planar regions
      if (commandInputManager.isNewCommandAvailable(HeightMapCommand.class))
      {
         latestHeightMap.set(commandInputManager.pollNewestCommand(HeightMapCommand.class));
      }
      commandInputManager.clearCommands(HeightMapCommand.class);

      // if the robot is standing, or we just finished a step, we should submit the newest regions
      if (latestWalkingStatus.get() == WalkingStatus.COMPLETED || latestFootstepStatusReceived.get() == FootstepStatus.COMPLETED)
         shouldSubmitNewRegions.set(true);

      // if the contact state just changed, we should submit the newest regions
      if (latestFootstepStatusReceived.get() != previousFootstepStatusReceived.get())
         shouldSubmitNewRegions.set(true);

      // submit the new planar regions
      if (isOpen && shouldSubmitNewRegions.getAndSet(false) && latestHeightMap.get() != null)
      {
         HeightMapCommand command = latestHeightMap.getAndSet(null);

         if (command != null)
         {
            for (int i = 0; i < heightMapCommandConsumers.size(); i++)
               heightMapCommandConsumers.get(i).accept(command);

            ticksSinceUpdatingTheEnvironment.set(0);
         }
      }

      ticksSinceUpdatingTheEnvironment.incrementAndGet();

      previousWalkingStatus.set(latestWalkingStatus.get());
      previousFootstepStatusReceived.set(latestFootstepStatusReceived.get());

      if (!isOpen)
      {
         walk.set(false);
         commandInputManager.clearAllCommands();
      }
   }

   public boolean isOpen()
   {
      return isOpen;
   }

   public DesiredVelocityProvider createDesiredVelocityProvider()
   {
      return new DesiredVelocityProvider()
      {
         @Override
         public Vector2DReadOnly getDesiredVelocity()
         {
            return desiredVelocity;
         }

         @Override
         public boolean areVelocitiesNormalized()
         {
            return areVelocitiesNormalized.getBooleanValue();
         }
      };
   }

   public DesiredTurningVelocityProvider createDesiredTurningVelocityProvider()
   {
      return new DesiredTurningVelocityProvider()
      {
         @Override
         public double getTurningVelocity()
         {
            return turningVelocity.getDoubleValue();
         }

         @Override
         public boolean areVelocitiesNormalized()
         {
            return areVelocitiesNormalized.getBooleanValue();
         }
      };
   }

   public BooleanProvider createWalkInputProvider()
   {
      return walk::getBooleanValue;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public void destroy()
   {
      ros2Node.destroy();
      heartbeatMonitor.destroy();
   }
}
