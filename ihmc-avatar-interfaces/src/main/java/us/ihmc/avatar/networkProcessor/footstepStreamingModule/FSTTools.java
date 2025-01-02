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

   public static void computeLinearVelocity(double dt,
                                            FramePoint3DReadOnly previousPosition,
                                            FramePoint3DReadOnly currentPosition,
                                            FixedFrameVector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.sub(currentPosition, previousPosition);
      linearVelocityToPack.scale(1.0 / dt);
   }

   /**
    * Computes the angular velocity from finite difference. The result is the angular velocity
    * expressed in the local frame described by {@code currentOrientation}.
    */
   public static void computeAngularVelocity(double dt,
                                             FrameQuaternionReadOnly previousOrientation,
                                             FrameQuaternionReadOnly currentOrientation,
                                             FixedFrameVector3DBasics angularVelocityToPack)
   {
      previousOrientation.checkReferenceFrameMatch(currentOrientation);
      previousOrientation.checkReferenceFrameMatch(angularVelocityToPack);

      double qDot_x = currentOrientation.getX() - previousOrientation.getX();
      double qDot_y = currentOrientation.getY() - previousOrientation.getY();
      double qDot_z = currentOrientation.getZ() - previousOrientation.getZ();
      double qDot_s = currentOrientation.getS() - previousOrientation.getS();

      double qx = -currentOrientation.getX();
      double qy = -currentOrientation.getY();
      double qz = -currentOrientation.getZ();
      double qs = currentOrientation.getS();

      double wx = qs * qDot_x + qx * qDot_s + qy * qDot_z - qz * qDot_y;
      double wy = qs * qDot_y - qx * qDot_z + qy * qDot_s + qz * qDot_x;
      double wz = qs * qDot_z + qx * qDot_y - qy * qDot_x + qz * qDot_s;
      angularVelocityToPack.set(wx, wy, wz);
      angularVelocityToPack.scale(2.0 / dt);
   }

   /**
    * Computes the acceleration from finite difference. This method uses two consecutive
    * velocity measurements and the time interval between them to approximate the acceleration.
    *
    * @param dt                         the time delta between the two velocity measurements
    * @param previousVelocity           the velocity measured at the previous time step
    * @param currentVelocity            the velocity measured at the current time step
    * @param accelerationToPack   the acceleration to pack, expressed in the same reference frame
    */
   public void computeAcceleration(double dt,
                                          FrameVector3DReadOnly previousVelocity,
                                          FrameVector3DReadOnly currentVelocity,
                                          FixedFrameVector3DBasics accelerationToPack)
   {
      // Ensure all arguments are in the same reference frame
      previousVelocity.checkReferenceFrameMatch(currentVelocity);
      previousVelocity.checkReferenceFrameMatch(accelerationToPack);

      accelerationToPack.sub(currentVelocity, previousVelocity);
      accelerationToPack.scale(1.0 / dt);
   }

   public static void integrateLinearVelocity(double dt,
                                              FramePoint3DReadOnly initialPosition,
                                              FrameVector3DReadOnly linearVelocity,
                                              FixedFramePoint3DBasics finalPosition)
   {
      finalPosition.scaleAdd(dt, linearVelocity, initialPosition);
   }

   public static void integrateAngularVelocity(double dt,
                                               FrameQuaternionReadOnly initialOrientation,
                                               FrameVector3DReadOnly angularVelocity,
                                               boolean isAngularVelocityLocal,
                                               FixedFrameQuaternionBasics finalOrientation)
   {
      double qInit_x = initialOrientation.getX();
      double qInit_y = initialOrientation.getY();
      double qInit_z = initialOrientation.getZ();
      double qInit_s = initialOrientation.getS();

      double x = angularVelocity.getX() * dt;
      double y = angularVelocity.getY() * dt;
      double z = angularVelocity.getZ() * dt;
      finalOrientation.setRotationVector(x, y, z);

      double qInt_x = finalOrientation.getX();
      double qInt_y = finalOrientation.getY();
      double qInt_z = finalOrientation.getZ();
      double qInt_s = finalOrientation.getS();

      if (isAngularVelocityLocal)
         QuaternionTools.multiplyImpl(qInit_x, qInit_y, qInit_z, qInit_s, false, qInt_x, qInt_y, qInt_z, qInt_s, false, finalOrientation);
      else
         QuaternionTools.multiplyImpl(qInt_x, qInt_y, qInt_z, qInt_s, false, qInit_x, qInit_y, qInit_z, qInit_s, false, finalOrientation);
   }
}
