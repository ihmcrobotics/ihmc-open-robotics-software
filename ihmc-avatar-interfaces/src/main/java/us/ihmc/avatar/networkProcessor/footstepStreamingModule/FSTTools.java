package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI.FootstepStreamingToolboxInputCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class FSTTools
{
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final FootstepStreamingToolboxParameters parameters;
   private final DoubleProvider time;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoRegistry registry;

   private final double toolboxControllerPeriod;
   private final YoLong currentMessageId;

   private final YoBoolean hasNewInputCommand, hasPreviousInput;
   private final YoDouble latestInputReceivedTime, previousInputReceivedTime;
   private FootstepStreamingToolboxInputCommand latestInput = null;
   private FootstepStreamingToolboxInputCommand previousInput = null;

   private final YoLong latestInputTimestampSource;
   private final YoDouble latestInputTimeSource;

   public FSTTools(CommandInputManager commandInputManager,
                   StatusMessageOutputManager statusOutputManager,
                   FootstepStreamingToolboxParameters parameters,
                   DoubleProvider time,
                   YoGraphicsListRegistry yoGraphicsListRegistry,
                   YoRegistry registry)
   {
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      this.parameters = parameters;
      this.toolboxControllerPeriod = parameters.getToolboxUpdatePeriod();
      this.time = time;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.registry = registry;

      currentMessageId = new YoLong("currentMessageId", registry);
      currentMessageId.set(1L);

      hasNewInputCommand = new YoBoolean("hasNewInputCommand", registry);
      hasPreviousInput = new YoBoolean("hasPreviousInput", registry);
      latestInputReceivedTime = new YoDouble("latestInputReceivedTime", registry);
      previousInputReceivedTime = new YoDouble("previousInputReceivedTime", registry);
      flushInputCommands();

      latestInputTimestampSource = new YoLong("latestInputTimestampSource", registry);
      latestInputTimeSource = new YoDouble("latestInputTimeSource", registry);
   }

   public void update()
   {
      if (commandInputManager.isNewCommandAvailable(FootstepStreamingToolboxInputCommand.class))
      {
         if (latestInput != null)
         {
            if (previousInput == null)
               previousInput = new FootstepStreamingToolboxInputCommand();

            previousInput.set(latestInput);
            previousInputReceivedTime.set(latestInputReceivedTime.getValue());
            hasPreviousInput.set(true);
         }

         if (latestInput == null)
            latestInput = new FootstepStreamingToolboxInputCommand();

         latestInput.set(commandInputManager.pollNewestCommand(FootstepStreamingToolboxInputCommand.class));

         latestInputTimestampSource.set(latestInput.getInputFor(RobotSide.LEFT).getTimestamp());
         latestInputTimeSource.set(latestInput.getInputFor(RobotSide.LEFT).getTimestamp() * 1.0e-9);

         if (latestInput.getInputFor(RobotSide.LEFT).getTimestamp() <= 0)
            latestInput.getInputFor(RobotSide.LEFT).setTimestamp(Conversions.secondsToNanoseconds(time.getValue()));

         latestInputReceivedTime.set(time.getValue());
         hasNewInputCommand.set(true);
      }
      else
      {
         hasNewInputCommand.set(false);
      }
   }

   public FootstepStreamingToolboxParameters getParameters()
   {
      return parameters;
   }

   public double getTime()
   {
      return time.getValue();
   }

   public FootstepStreamingToolboxInputCommand getLatestInput()
   {
      return latestInput;
   }

   public double getLatestInputReceivedTime()
   {
      return latestInputReceivedTime.getValue();
   }

   public boolean hasPreviousInput()
   {
      return hasPreviousInput.getValue();
   }

   public FootstepStreamingToolboxInputCommand getPreviousInput()
   {
      return hasPreviousInput.getValue() ? previousInput : null;
   }

   public double getPreviousInputReceivedTime()
   {
      return previousInputReceivedTime.getValue();
   }

   public boolean hasNewInputCommand()
   {
      return hasNewInputCommand.getValue();
   }

   public void flushInputCommands()
   {
      latestInput = null;
      commandInputManager.clearAllCommands();
      hasNewInputCommand.set(false);
      hasPreviousInput.set(false);
      latestInputReceivedTime.set(-1.0);
      previousInputReceivedTime.set(-1.0);
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public StatusMessageOutputManager getStatusOutputManager()
   {
      return statusOutputManager;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public double getToolboxControllerPeriod()
   {
      return toolboxControllerPeriod;
   }
}
