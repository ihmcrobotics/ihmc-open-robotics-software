package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;
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

   private final TimestampProvider monotonicTimeProvider;

   private final List<ValkyrieRosControlEffortJointControlCommandCalculator> effortControlCommandCalculators = new ArrayList<>();
   private final LinkedHashMap<String, ValkyrieRosControlEffortJointControlCommandCalculator> effortJointToControlCommandCalculatorMap = new LinkedHashMap<>();

   private final List<ValkyrieRosControlPositionJointControlCommandCalculator> positionControlCommandCalculators = new ArrayList<>();

   private final YoDouble yoTime = new YoDouble("lowLevelControlTime", registry);
   private final YoDouble wakeUpTime = new YoDouble("lowLevelControlWakeUpTime", registry);

   private final ValkyrieFingerController fingerController;

   private final AtomicReference<HighLevelControllerName> currentHighLevelControllerState = new AtomicReference<HighLevelControllerName>(null);

   private JointTorqueOffsetEstimator jointTorqueOffsetEstimator;

   public ValkyrieRosControlLowLevelController(TimestampProvider monotonicTimeProvider, final double updateDT,
                                               ValkyrieRosControlFingerStateEstimator fingerStateEstimator,
                                               List<YoEffortJointHandleHolder> yoEffortJointHandleHolders,
                                               List<YoPositionJointHandleHolder> yoPositionJointHandleHolders, ValkyrieJointMap jointMap,
                                               YoVariableRegistry parentRegistry)
   {
      this.monotonicTimeProvider = monotonicTimeProvider;

      wakeUpTime.set(Double.NaN);

      if (ValkyrieRosControlController.ENABLE_FINGER_JOINTS)
         fingerController = new ValkyrieFingerController(yoTime, updateDT, fingerStateEstimator, yoEffortJointHandleHolders, registry);
      else
         fingerController = null;

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

      for (YoPositionJointHandleHolder positionJointHandleHolder : yoPositionJointHandleHolders)
      {
         ValkyrieRosControlPositionJointControlCommandCalculator controlCommandCalculator = new ValkyrieRosControlPositionJointControlCommandCalculator(positionJointHandleHolder,
                                                                                                                                                        registry);
         positionControlCommandCalculators.add(controlCommandCalculator);
      }

      parentRegistry.addChild(registry);
   }

   public void doControl()
   {
      long timestamp = monotonicTimeProvider.getTimestamp();

      if (wakeUpTime.isNaN())
         wakeUpTime.set(Conversions.nanosecondsToSeconds(timestamp));

      yoTime.set(Conversions.nanosecondsToSeconds(timestamp) - wakeUpTime.getDoubleValue());

      if (ValkyrieRosControlController.ENABLE_FINGER_JOINTS)
         fingerController.doControl();
      updateCommandCalculators();
   }

   public void updateCommandCalculators()
   {
      for (int i = 0; i < effortControlCommandCalculators.size(); i++)
         effortControlCommandCalculators.get(i).computeAndUpdateJointTorque();
      for (int i = 0; i < positionControlCommandCalculators.size(); i++)
         positionControlCommandCalculators.get(i).computeAndUpdateJointPosition();
   }

   private void writeTorqueOffsets()
   {
      List<OneDoFJointBasics> oneDoFJoints = jointTorqueOffsetEstimator.getOneDoFJoints();
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJointBasics joint = oneDoFJoints.get(i);
         if (jointTorqueOffsetEstimator.hasTorqueOffsetForJoint(joint))
         {
            subtractTorqueOffset(joint, jointTorqueOffsetEstimator.getEstimatedJointTorqueOffset(joint));
            jointTorqueOffsetEstimator.resetEstimatedJointTorqueOffset(joint);
         }
      }
   }

   public void subtractTorqueOffset(OneDoFJointBasics oneDoFJoint, double torqueOffset)
   {
      ValkyrieRosControlEffortJointControlCommandCalculator jointCommandCalculator = effortJointToControlCommandCalculatorMap.get(oneDoFJoint.getName());
      if (jointCommandCalculator != null)
         jointCommandCalculator.subtractTorqueOffset(torqueOffset);
      else
         LogTools.error("Command calculator is NULL for the joint: " + oneDoFJoint.getName());
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
               currentHighLevelControllerState.set(HighLevelControllerName.fromByte(statusMessage.getEndHighLevelControllerName()));

               if (statusMessage.getInitialHighLevelControllerName() == HighLevelControllerName.CALIBRATION.toByte())
                  writeTorqueOffsets();

               if (ValkyrieRosControlController.ENABLE_FINGER_JOINTS)
               {
                  if (statusMessage.getEndHighLevelControllerName() == HighLevelControllerName.CALIBRATION.toByte())
                     fingerController.goToInitialConfiguration();
               }
            }
         }
      };
      statusOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, highLevelStateChangeListener);
   }

   public void attachJointTorqueOffsetEstimator(JointTorqueOffsetEstimator jointTorqueOffsetEstimator)
   {
      this.jointTorqueOffsetEstimator = jointTorqueOffsetEstimator;
   }

   public void setupLowLevelControlCommunication(String robotName, RealtimeRos2Node realtimeRos2Node)
   {
      if (ValkyrieRosControlController.ENABLE_FINGER_JOINTS)
         fingerController.setupCommunication(robotName, realtimeRos2Node);
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
