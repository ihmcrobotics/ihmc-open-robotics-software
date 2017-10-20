package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.TimestampProvider;
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

   private final ValkyrieTorqueHysteresisCompensator torqueHysteresisCompensator;

   private final AtomicReference<HighLevelController> currentHighLevelControllerState = new AtomicReference<HighLevelController>(null);

   private JointTorqueOffsetEstimator jointTorqueOffsetEstimator;

   public ValkyrieRosControlLowLevelController(TimestampProvider timestampProvider, final double updateDT,
                                               List<YoEffortJointHandleHolder> yoEffortJointHandleHolders,
                                               List<YoPositionJointHandleHolder> yoPositionJointHandleHolders, YoVariableRegistry parentRegistry)
   {
      this.timestampProvider = timestampProvider;

      wakeUpTime.set(Double.NaN);

      torqueHysteresisCompensator = new ValkyrieTorqueHysteresisCompensator(yoEffortJointHandleHolders, yoTime, registry);

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

      if (currentHighLevelControllerState.get() == HighLevelController.WALKING)
      {
         torqueHysteresisCompensator.compute();
      }

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

               if (statusMessage.initialState == HighLevelController.CALIBRATION)
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
   }
}
