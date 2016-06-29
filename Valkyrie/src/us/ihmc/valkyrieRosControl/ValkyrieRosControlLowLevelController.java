package us.ihmc.valkyrieRosControl;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;

public class ValkyrieRosControlLowLevelController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final TimestampProvider timestampProvider;

   private final AtomicBoolean resetIHMCControlRatioAndStandPrepRequested = new AtomicBoolean(false);

   private final ArrayList<ValkyrieRosControlEffortJointControlCommandCalculator> effortControlCommandCalculators = new ArrayList<>();
   private final LinkedHashMap<String, ValkyrieRosControlEffortJointControlCommandCalculator> effortJointToControlCommandCalculatorMap = new LinkedHashMap<>();
   private final ArrayList<ValkyrieRosControlPositionJointControlCommandCalculator> positionControlCommandCalculators = new ArrayList<>();
   private final LinkedHashMap<String, ValkyrieRosControlPositionJointControlCommandCalculator> positionJointToControlCommandCalculatorMap = new LinkedHashMap<>();

   private final DoubleYoVariable doIHMCControlRatio;
   private final DoubleYoVariable timeInStandprep;
   private final DoubleYoVariable standPrepRampUpTime;
   private final DoubleYoVariable masterGain;

   private long standPrepStartTime = -1;

   private final ValkyrieTorqueHysteresisCompensator torqueHysteresisCompensator;
   private final ValkyrieAccelerationIntegration accelerationIntegration;

   @SuppressWarnings("unchecked")
   public ValkyrieRosControlLowLevelController(TimestampProvider timestampProvider, double updateDT, List<YoEffortJointHandleHolder> yoEffortJointHandleHolders,
         List<YoPositionJointHandleHolder> yoPositionJointHandleHolders, YoVariableRegistry parentRegistry)
   {
      this.timestampProvider = timestampProvider;

      doIHMCControlRatio = new DoubleYoVariable("doIHMCControlRatio", registry);
      masterGain = new DoubleYoVariable("StandPrepMasterGain", registry);
      timeInStandprep = new DoubleYoVariable("timeInStandprep", registry);
      standPrepRampUpTime = new DoubleYoVariable("standPrepRampUpTime", registry);
      standPrepRampUpTime.set(5.0);
      masterGain.set(0.3);

      torqueHysteresisCompensator = new ValkyrieTorqueHysteresisCompensator(yoEffortJointHandleHolders, timeInStandprep, registry);
      accelerationIntegration = new ValkyrieAccelerationIntegration(yoEffortJointHandleHolders, updateDT, registry);

      Yaml yaml = new Yaml();

      InputStream gainStream = getClass().getClassLoader().getResourceAsStream("standPrep/gains.yaml");
      InputStream setpointsStream = getClass().getClassLoader().getResourceAsStream("standPrep/setpoints.yaml");
      InputStream offsetsStream;
      try
      {
         offsetsStream = new FileInputStream(new File(ValkyrieTorqueOffsetPrinter.IHMC_TORQUE_OFFSET_FILE));
      }
      catch (FileNotFoundException e1)
      {
         offsetsStream = null;
      }

      Map<String, Map<String, Double>> gainMap = (Map<String, Map<String, Double>>) yaml.load(gainStream);
      Map<String, Double> setPointMap = (Map<String, Double>) yaml.load(setpointsStream);
      Map<String, Double> offsetMap = null;
      if (offsetsStream != null)
         offsetMap = (Map<String, Double>) yaml.load(offsetsStream);

      try
      {
         gainStream.close();
         setpointsStream.close();
         if (offsetsStream != null)
            offsetsStream.close();
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
            torqueOffset = offsetMap.get(jointName);

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

      for (YoPositionJointHandleHolder positionJointHandleHolder : yoPositionJointHandleHolders)
      {
         String jointName = positionJointHandleHolder.getName();
         Map<String, Double> standPrepGains = gainMap.get(jointName);

         double standPrepAngle = 0.0;
         if (setPointMap.containsKey(jointName))
         {
            standPrepAngle = setPointMap.get(jointName);
         }
         ValkyrieRosControlPositionJointControlCommandCalculator controlCommandCalculator = new ValkyrieRosControlPositionJointControlCommandCalculator(
               positionJointHandleHolder, standPrepGains, standPrepAngle, updateDT, registry);
         positionControlCommandCalculators.add(controlCommandCalculator);

         positionJointToControlCommandCalculatorMap.put(jointName, controlCommandCalculator);
      }

      parentRegistry.addChild(registry);
   }

   public void setDoIHMCControlRatio(double controlRatio)
   {
      doIHMCControlRatio.set(MathTools.clipToMinMax(controlRatio, 0.0, 1.0));
   }

   public void doControl()
   {
      long timestamp = timestampProvider.getTimestamp();

      if (resetIHMCControlRatioAndStandPrepRequested.getAndSet(false))
      {
         standPrepStartTime = -1;
         doIHMCControlRatio.set(0.0);
      }

      if (standPrepStartTime > 0)
      {
         timeInStandprep.set(TimeTools.nanoSecondstoSeconds(timestamp - standPrepStartTime));

         torqueHysteresisCompensator.compute();
         if (ValkyrieRosControlController.INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES)
            accelerationIntegration.compute();

         double ramp = timeInStandprep.getDoubleValue() / standPrepRampUpTime.getDoubleValue();
         ramp = MathTools.clipToMinMax(ramp, 0.0, 1.0);

         for (int i = 0; i < effortControlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlEffortJointControlCommandCalculator commandCalculator = effortControlCommandCalculators.get(i);
            commandCalculator.computeAndUpdateJointTorque(ramp, doIHMCControlRatio.getDoubleValue(), masterGain.getDoubleValue());
         }

         for (int i = 0; i < positionControlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlPositionJointControlCommandCalculator commandCalculator = positionControlCommandCalculators.get(i);
            commandCalculator.computeAndUpdateJointPosition(ramp, doIHMCControlRatio.getDoubleValue(), masterGain.getDoubleValue());
         }
      }
      else
      {
         standPrepStartTime = timestamp;
         for (int i = 0; i < effortControlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlEffortJointControlCommandCalculator commandCalculator = effortControlCommandCalculators.get(i);
            commandCalculator.initialize();
         }

         for (int i = 0; i < positionControlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlPositionJointControlCommandCalculator commandCalculator = positionControlCommandCalculators.get(i);
            commandCalculator.initialize();
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
      StatusMessageListener<WalkingControllerFailureStatusMessage> controllerFailureListener = new StatusMessageListener<WalkingControllerFailureStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(WalkingControllerFailureStatusMessage statusMessage)
         {
            if (statusMessage != null)
               resetIHMCControlRatioAndStandPrepRequested.set(true);
         }
      };
      statusOutputManager.attachStatusMessageListener(WalkingControllerFailureStatusMessage.class, controllerFailureListener);
   }
}
