package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrie.fingers.ValkyrieFingerController;
import us.ihmc.valkyrie.fingers.ValkyrieHandJointName;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieRosControlLowLevelController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final TimestampProvider timestampProvider;

   private final ArrayList<ValkyrieRosControlEffortJointControlCommandCalculator> effortControlCommandCalculators = new ArrayList<>();
   private final LinkedHashMap<String, ValkyrieRosControlEffortJointControlCommandCalculator> effortJointToControlCommandCalculatorMap = new LinkedHashMap<>();

   private final YoDouble yoTime = new YoDouble("lowLevelControlTime", registry);
   private final YoDouble wakeUpTime = new YoDouble("lowLevelControlWakeUpTime", registry);

   private final ValkyrieFingerController fingerController;

   private final AtomicReference<HighLevelControllerName> currentHighLevelControllerState = new AtomicReference<HighLevelControllerName>(null);

   private JointTorqueOffsetEstimator jointTorqueOffsetEstimator;

   public ValkyrieRosControlLowLevelController(TimestampProvider timestampProvider, final double updateDT,
                                               List<YoEffortJointHandleHolder> yoEffortJointHandleHolders,
                                               List<YoPositionJointHandleHolder> yoPositionJointHandleHolders, ValkyrieJointMap jointMap, YoVariableRegistry parentRegistry)
   {
      this.timestampProvider = timestampProvider;

      wakeUpTime.set(Double.NaN);

      fingerController = new ValkyrieFingerController(yoTime, updateDT, yoEffortJointHandleHolders, registry);

      // Remove the finger joints to let the finger controller be the only controlling them
      yoEffortJointHandleHolders = yoEffortJointHandleHolders.stream().filter(h -> !isFingerJoint(h)).collect(Collectors.toList());

      Map<String, Double> offsetMap = ValkyrieTorqueOffsetPrinter.loadTorqueOffsetsFromFile();

      for (YoEffortJointHandleHolder effortJointHandleHolder : yoEffortJointHandleHolders)
      {
         String jointName = effortJointHandleHolder.getName();
         double torqueOffset = 0.0;
         if (offsetMap != null && offsetMap.containsKey(jointName))
            torqueOffset = -offsetMap.get(jointName);

         ValkyrieRosControlEffortJointControlCommandCalculator controlCommandCalculator = new ValkyrieRosControlEffortJointControlCommandCalculator(effortJointHandleHolder,
                                                                                                                                                    torqueOffset,
                                                                                                                                                    updateDT,
                                                                                                                                                    registry);
         effortControlCommandCalculators.add(controlCommandCalculator);

         effortJointToControlCommandCalculatorMap.put(jointName, controlCommandCalculator);
      }

      parentRegistry.addChild(registry);
   }

   public void doControl()
   {
      long timestamp = timestampProvider.getTimestamp();

      if (wakeUpTime.isNaN())
         wakeUpTime.set(Conversions.nanosecondsToSeconds(timestamp));

      yoTime.set(Conversions.nanosecondsToSeconds(timestamp) - wakeUpTime.getDoubleValue());

      fingerController.doControl();
      updateCommandCalculators();
   }

   public void updateCommandCalculators()
   {
      for (int i = 0; i < effortControlCommandCalculators.size(); i++)
      {
         ValkyrieRosControlEffortJointControlCommandCalculator commandCalculator = effortControlCommandCalculators.get(i);
         commandCalculator.computeAndUpdateJointTorque();
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
      StatusMessageListener<HighLevelStateChangeStatusMessage> highLevelStateChangeListener = new StatusMessageListener<HighLevelStateChangeStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(HighLevelStateChangeStatusMessage statusMessage)
         {
            if (statusMessage != null)
            {
               currentHighLevelControllerState.set(statusMessage.endState);

               if (statusMessage.initialState == HighLevelControllerName.CALIBRATION)
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
      fingerController.setupCommunication(packetCommunicator);
   }

   /**
    * Very inefficient way of checking if a joint is a finger joint that should be used only at construction time.
    */
   private static boolean isFingerJoint(YoEffortJointHandleHolder handle)
   {
      for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (handle.getName().equals(valkyrieHandJointName.getJointName(robotSide)))
               return true;
         }
      }
      return false;
   }
}
