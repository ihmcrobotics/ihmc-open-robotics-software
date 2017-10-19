package us.ihmc.valkyrieRosControl;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.humanoidRobotics.communication.packets.valkyrie.ValkyrieLowLevelControlModeMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.taskExecutor.TaskExecutor;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimator;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class ValkyrieRosControlLowLevelController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final TimestampProvider timestampProvider;

   private final ArrayList<ValkyrieRosControlEffortJointControlCommandCalculator> effortControlCommandCalculators = new ArrayList<>();
   private final LinkedHashMap<String, ValkyrieRosControlEffortJointControlCommandCalculator> effortJointToControlCommandCalculatorMap = new LinkedHashMap<>();

   private final YoDouble yoTime = new YoDouble("lowLevelControlTime", registry);
   private final YoDouble wakeUpTime = new YoDouble("lowLevelControlWakeUpTime", registry);

   private final YoDouble doIHMCControlRatio = new YoDouble("doIHMCControlRatio", registry);
   private final YoDouble masterGain = new YoDouble("standPrepMasterGain", registry);

   private final YoEnum<ValkyrieLowLevelControlModeMessage.ControlMode> requestedLowLevelControlMode = new YoEnum<>("requestedLowLevelControlMode", registry, ValkyrieLowLevelControlModeMessage.ControlMode.class, true);
   private final AtomicReference<ValkyrieLowLevelControlModeMessage.ControlMode> requestedLowLevelControlModeAtomic = new AtomicReference<>(null);

   private final ValkyrieTorqueHysteresisCompensator torqueHysteresisCompensator;
   private final ValkyrieAccelerationIntegration accelerationIntegration;

   private CommandInputManager commandInputManager;
   private final AtomicReference<HighLevelController> previousHighLevelControllerState = new AtomicReference<HighLevelController>(null);
   private final AtomicReference<HighLevelController> currentHighLevelControllerState = new AtomicReference<HighLevelController>(null);

   private final HighLevelControllerStateCommand highLevelControllerStateCommand = new HighLevelControllerStateCommand();

   private final TaskExecutor taskExecutor = new TaskExecutor();

   private JointTorqueOffsetEstimator jointTorqueOffsetEstimator;

   @SuppressWarnings("unchecked")
   public ValkyrieRosControlLowLevelController(TimestampProvider timestampProvider, final double updateDT, List<YoEffortJointHandleHolder> yoEffortJointHandleHolders,
         List<YoPositionJointHandleHolder> yoPositionJointHandleHolders, YoVariableRegistry parentRegistry)
   {
      this.timestampProvider = timestampProvider;

      masterGain.set(0.3);

      wakeUpTime.set(Double.NaN);

      torqueHysteresisCompensator = new ValkyrieTorqueHysteresisCompensator(yoEffortJointHandleHolders, yoTime, registry);
      accelerationIntegration = new ValkyrieAccelerationIntegration(yoEffortJointHandleHolders, updateDT, registry);

      requestedLowLevelControlMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (requestedLowLevelControlMode.getEnumValue() != null)
               requestedLowLevelControlModeAtomic.set(requestedLowLevelControlMode.getEnumValue());
         }
      });

      Yaml yaml = new Yaml();
      InputStream gainStream = getClass().getClassLoader().getResourceAsStream("standPrep/gains.yaml");
      InputStream setpointsStream = getClass().getClassLoader().getResourceAsStream("standPrep/setpoints.yaml");

      Map<String, Map<String, Double>> gainMap = (Map<String, Map<String, Double>>) yaml.load(gainStream);
      Map<String, Double> setPointMap = (Map<String, Double>) yaml.load(setpointsStream);
      Map<String, Double> offsetMap = ValkyrieTorqueOffsetPrinter.loadTorqueOffsetsFromFile();

      try
      {
         gainStream.close();
         setpointsStream.close();
      }
      catch (IOException e)
      {
      }

      for (YoEffortJointHandleHolder effortJointHandleHolder : yoEffortJointHandleHolders)
      {
         String jointName = effortJointHandleHolder.getName();
         Map<String, Double> standPrepGains = gainMap.get(jointName);
         double torqueOffset = 0.0;
         if (offsetMap != null && offsetMap.containsKey(jointName))
            torqueOffset = -offsetMap.get(jointName);

         double standPrepAngle = 0.0;
         if (setPointMap.containsKey(jointName))
         {
            standPrepAngle = setPointMap.get(jointName);
         }
         ValkyrieRosControlEffortJointControlCommandCalculator controlCommandCalculator = new ValkyrieRosControlEffortJointControlCommandCalculator(
               effortJointHandleHolder, standPrepGains, torqueOffset, standPrepAngle, updateDT, registry);
         effortControlCommandCalculators.add(controlCommandCalculator);

         effortJointToControlCommandCalculatorMap.put(jointName, controlCommandCalculator);
      }

      parentRegistry.addChild(registry);
   }

   public void setDoIHMCControlRatio(double controlRatio)
   {
      doIHMCControlRatio.set(MathTools.clamp(controlRatio, 0.0, 1.0));
   }

   public void doControl()
   {
      long timestamp = timestampProvider.getTimestamp();

      if (wakeUpTime.isNaN())
         wakeUpTime.set(Conversions.nanosecondsToSeconds(timestamp));

      yoTime.set(Conversions.nanosecondsToSeconds(timestamp) - wakeUpTime.getDoubleValue());

      taskExecutor.doControl();
      ValkyrieLowLevelControlModeMessage.ControlMode newRequest = requestedLowLevelControlModeAtomic.getAndSet(null);
      requestedLowLevelControlMode.set(null);

      switch (currentHighLevelControllerState.get())
      {
      case STAND_READY:
         if (newRequest == null)
            break;

         switch (newRequest)
         {
         case CALIBRATION:
            highLevelControllerStateCommand.setHighLevelController(HighLevelController.CALIBRATION);
            commandInputManager.submitCommand(highLevelControllerStateCommand);
            break;
         case HIGH_LEVEL_CONTROL:
            highLevelControllerStateCommand.setHighLevelController(HighLevelController.STAND_TRANSITION_STATE);
            commandInputManager.submitCommand(highLevelControllerStateCommand);
            break;
         default:
            break;
         }
         break;
      case STAND_TRANSITION_STATE:
      case WALKING:
         if (newRequest != null && newRequest == ValkyrieLowLevelControlModeMessage.ControlMode.STAND_PREP)
         {
            highLevelControllerStateCommand.setHighLevelController(HighLevelController.STAND_PREP_STATE);
            commandInputManager.submitCommand(highLevelControllerStateCommand);
            break;
         }

         torqueHysteresisCompensator.compute();
         if (ValkyrieRosControlController.INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES)
            accelerationIntegration.compute();
         break;
      default:
         break;

      }

      updateCommandCalculators();
   }

   public void updateCommandCalculators()
   {
      for (int i = 0; i < effortControlCommandCalculators.size(); i++)
      {
         ValkyrieRosControlEffortJointControlCommandCalculator commandCalculator = effortControlCommandCalculators.get(i);
         commandCalculator.computeAndUpdateJointTorque(doIHMCControlRatio.getDoubleValue(), masterGain.getDoubleValue());
      }
   }

   private void writeTorqueOffsets()
   {
      List<OneDoFJoint> oneDoFJoints = jointTorqueOffsetEstimator.getOneDoFJoints();
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJoint joint = oneDoFJoints.get(i);
         if (jointTorqueOffsetEstimator.hasTorqueOffsetForJoint(joint))
         {
            subtractTorqueOffset(joint, jointTorqueOffsetEstimator.getEstimatedJointTorqueOffset(joint));
            jointTorqueOffsetEstimator.resetEstimatedJointTorqueOffset(joint);
         }
      }
   }

   public void subtractTorqueOffset(OneDoFJoint oneDoFJoint, double torqueOffset)
   {
      ValkyrieRosControlEffortJointControlCommandCalculator jointCommandCalculator = effortJointToControlCommandCalculatorMap.get(oneDoFJoint.getName());
      if (jointCommandCalculator != null)
         jointCommandCalculator.subtractTorqueOffset(torqueOffset);
      else
         PrintTools.error("Command calculator is NULL for the joint: " + oneDoFJoint.getName());
   }

   public void attachControllerAPI(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager)
   {
      this.commandInputManager = commandInputManager;

      StatusMessageListener<HighLevelStateChangeStatusMessage> highLevelStateChangeListener = new StatusMessageListener<HighLevelStateChangeStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(HighLevelStateChangeStatusMessage statusMessage)
         {
            if (statusMessage != null)
            {
               previousHighLevelControllerState.set(statusMessage.initialState);
               currentHighLevelControllerState.set(statusMessage.endState);

               if (previousHighLevelControllerState.get() == HighLevelController.CALIBRATION)
                  writeTorqueOffsets();
            }
         }
      };
      statusOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, highLevelStateChangeListener);
   }
   
   public void attachJointTorqueOffsetEstimator(JointTorqueOffsetEstimator jointTorqueOffsetEstimator)
   {
      this.jointTorqueOffsetEstimator = jointTorqueOffsetEstimator;
   }

   public void setupLowLevelControlWithPacketCommunicator(PacketCommunicator packetCommunicator)
   {
      packetCommunicator.attachListener(ValkyrieLowLevelControlModeMessage.class, new PacketConsumer<ValkyrieLowLevelControlModeMessage>()
      {
         @Override
         public void receivedPacket(ValkyrieLowLevelControlModeMessage packet)
         {
            if (packet != null && packet.getRequestedControlMode() != null)
               requestedLowLevelControlModeAtomic.set(packet.getRequestedControlMode());
         }
      });
   }
}
