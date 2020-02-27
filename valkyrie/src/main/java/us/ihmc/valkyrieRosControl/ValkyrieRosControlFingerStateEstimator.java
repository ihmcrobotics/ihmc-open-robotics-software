package us.ihmc.valkyrieRosControl;

import static us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName.IndexFingerMotorPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName.MiddleFingerMotorPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName.PinkyMotorPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName.ThumbMotorPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName.ThumbMotorPitch2;
import static us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName.ThumbMotorRoll;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.IndexFingerPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.IndexFingerPitch2;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.IndexFingerPitch3;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.MiddleFingerPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.MiddleFingerPitch2;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.MiddleFingerPitch3;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.PinkyPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.PinkyPitch2;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.PinkyPitch3;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.ThumbPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.ThumbPitch2;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.ThumbPitch3;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.ThumbRoll;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.yaml.snakeyaml.Yaml;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName;
import us.ihmc.valkyrie.fingers.ValkyrieHandJointName;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoJointStateHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class ValkyrieRosControlFingerStateEstimator implements SensorProcessingConfiguration
{
   private static final String FINGER_TRANSMISSION_FILE = System.getProperty("user.home") + File.separator
         + "valkyrie/ValkyrieFingerJointTransmissionCoeffs.yaml";

   private final SensorProcessingConfiguration sensorProcessingConfiguration;
   private final SideDependentList<EnumMap<ValkyrieHandJointName, Mean>> sideDependentFingerPositionMeans = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoBoolean>> sideDependentEncoderDeadMap = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentScales = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentBiases = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, DoubleProvider>> sideDependentMotorBasedFingerJointPositions = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder>> sideDependentFingerMotorHandles = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, OneDoFJointBasics>> sideDependentFingerJoints = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);

   private final YoBoolean forceMotorBasedPositionSwitch;
   private final YoEnum<RobotSide> doZeroFingerCalibrationNow;

   private final YoBoolean startFingerCalibration;
   private final YoBoolean isCalibratingFingers;
   private final TimestampProvider monotonicTimeProvider;
   private final YoDouble fingerCalibrationStartTime;
   private final YoDouble fingerCalibrationDuration;

   public ValkyrieRosControlFingerStateEstimator(List<YoEffortJointHandleHolder> yoEffortJointHandleHolders,
                                                 List<YoPositionJointHandleHolder> yoPositionJointHandleHolders,
                                                 List<YoJointStateHandleHolder> yoJointStateHandleHolders, TimestampProvider monotonicTimeProvider,
                                                 StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
                                                 SensorProcessingConfiguration sensorProcessingConfiguration, YoVariableRegistry registry)
   {
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.sensorProcessingConfiguration = sensorProcessingConfiguration;
      List<OneDoFJointBasics> allJoints = stateEstimatorSensorDefinitions.getJointSensorDefinitions();

      forceMotorBasedPositionSwitch = new YoBoolean("forceMotorBasedPositionSwitch", registry);
      doZeroFingerCalibrationNow = new YoEnum<>("doZeroFingerCalibrationNow", registry, RobotSide.class, true);
      doZeroFingerCalibrationNow.set(null);
      startFingerCalibration = new YoBoolean("startFingerCalibration", registry);
      isCalibratingFingers = new YoBoolean("isCalibratingFingers", registry);
      fingerCalibrationStartTime = new YoDouble("fingerCalibrationStartTime", registry);
      fingerCalibrationDuration = new YoDouble("fingerCalibrationDuration", registry);
      fingerCalibrationDuration.set(1.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, Mean> fingerPositionMeans = sideDependentFingerPositionMeans.get(robotSide);
         EnumMap<ValkyrieHandJointName, YoDouble> scales = sideDependentScales.get(robotSide);
         EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiases.get(robotSide);
         EnumMap<ValkyrieHandJointName, YoBoolean> encoderDeadMap = sideDependentEncoderDeadMap.get(robotSide);
         EnumMap<ValkyrieHandJointName, OneDoFJointBasics> fingerJoints = sideDependentFingerJoints.get(robotSide);

         for (ValkyrieHandJointName fingerJoint : ValkyrieHandJointName.values)
         {
            String jointName = fingerJoint.getPascalCaseJointName(robotSide);
            scales.put(fingerJoint, new YoDouble("scale" + jointName, registry));
            biases.put(fingerJoint, new YoDouble("bias" + jointName, registry));
            fingerPositionMeans.put(fingerJoint, new Mean());
            encoderDeadMap.put(fingerJoint, new YoBoolean("is" + jointName + "EncoderDead", registry));
            fingerJoints.put(fingerJoint, allJoints.stream().filter(j -> j.getName().equals(fingerJoint.getJointName(robotSide))).findFirst().get());
         }

         EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder> fingerMotorHandles = sideDependentFingerMotorHandles.get(robotSide);

         for (ValkyrieFingerMotorName motor : ValkyrieFingerMotorName.values)
         {
            fingerMotorHandles.put(motor, yoEffortJointHandleHolders.stream().filter(h -> h.getName().equals(motor.getJointName(robotSide))).findFirst().get());
         }
      }
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      sensorProcessingConfiguration.configureSensorProcessing(sensorProcessing);
      configureFingerProcessing(sensorProcessing);
   }

   private void configureFingerProcessing(SensorProcessing sensorProcessing)
   {
      YoVariableRegistry registry = sensorProcessing.getYoVariableRegistry();

      boolean areCoeffsLoaded = loadCoeffsFromFile(FINGER_TRANSMISSION_FILE, sideDependentScales, sideDependentBiases);

      if (!areCoeffsLoaded)
      {
         loadDefaultCoeffs(sideDependentScales, sideDependentBiases);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder> motorHandles = sideDependentFingerMotorHandles.get(robotSide);

         { // Doing the thumb separately
            ValkyrieFingerMotorName[] motors = {ThumbMotorRoll, ThumbMotorPitch1, ThumbMotorPitch2, ThumbMotorPitch2};
            ValkyrieHandJointName[] joints = {ThumbRoll, ThumbPitch1, ThumbPitch2, ThumbPitch3};

            for (int i = 0; i < 4; i++)
            {
               ValkyrieHandJointName slave = joints[i];
               YoDouble scale = sideDependentScales.get(robotSide).get(slave);
               YoDouble bias = sideDependentBiases.get(robotSide).get(slave);
               String slaveJointName = slave.getCamelCaseJointName(robotSide);
               YoEffortJointHandleHolder motorHandle = motorHandles.get(motors[i]);
               DoubleProvider motorBasedPosition = sensorProcessing.computeJointPositionUsingCoupling(motorHandle::getQ, slaveJointName, scale, bias, true);
               sideDependentMotorBasedFingerJointPositions.get(robotSide).put(slave, motorBasedPosition);
               Predicate<DoubleProvider> backupTrigger = createFingerJointPositionSwitchTrigger(robotSide, slave, registry);
               sensorProcessing.addJointPositionSensorSwitch(slaveJointName, motorBasedPosition, backupTrigger, false);
            }
         }

         ValkyrieFingerMotorName[] motors = {IndexFingerMotorPitch1, MiddleFingerMotorPitch1, PinkyMotorPitch1};
         ValkyrieHandJointName[][] slaveJoints = {{IndexFingerPitch1, MiddleFingerPitch1, PinkyPitch1}, {IndexFingerPitch2, MiddleFingerPitch2, PinkyPitch2},
               {IndexFingerPitch3, MiddleFingerPitch3, PinkyPitch3}};

         for (int fingerIndex = 0; fingerIndex < 3; fingerIndex++)
         {
            YoEffortJointHandleHolder motorHandle = motorHandles.get(motors[fingerIndex]);

            for (int i = 0; i < slaveJoints.length; i++)
            {
               ValkyrieHandJointName slave = slaveJoints[i][fingerIndex];
               YoDouble scale = sideDependentScales.get(robotSide).get(slave);
               YoDouble bias = sideDependentBiases.get(robotSide).get(slave);
               String slaveJointName = slave.getCamelCaseJointName(robotSide);
               DoubleProvider motorBasedPosition = sensorProcessing.computeJointPositionUsingCoupling(motorHandle::getQ, slaveJointName, scale, bias, true);
               sideDependentMotorBasedFingerJointPositions.get(robotSide).put(slave, motorBasedPosition);
               Predicate<DoubleProvider> backupTrigger = createFingerJointPositionSwitchTrigger(robotSide, slave, registry);
               sensorProcessing.addJointPositionSensorSwitch(slaveJointName, motorBasedPosition, backupTrigger, false);
            }
         }
      }
   }

   private boolean loadCoeffsFromFile(String fingerTransmissionFile, SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentScalesToLoad,
                                      SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentBiasesToLoad)
   {
      boolean areCoeffsLoaded = false;

      Yaml yaml = new Yaml();
      File coeffFile = new File(fingerTransmissionFile);

      if (coeffFile.exists())
      {
         try
         {
            FileInputStream fileInputStream = new FileInputStream(coeffFile);
            @SuppressWarnings("unchecked")
            Map<String, Map<String, Double>> coeffs = (Map<String, Map<String, Double>>) yaml.load(fileInputStream);

            for (RobotSide robotSide : RobotSide.values)
            {
               EnumMap<ValkyrieHandJointName, YoDouble> scales = sideDependentScalesToLoad.get(robotSide);
               EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiasesToLoad.get(robotSide);

               for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
               {
                  Map<String, Double> jointCoeffs = coeffs.get(jointEnum.getJointName(robotSide));

                  scales.get(jointEnum).set(jointCoeffs.getOrDefault("scale", 0.0));
                  biases.get(jointEnum).set(jointCoeffs.getOrDefault("bias", 0.0));
               }
            }
            areCoeffsLoaded = true;
         }
         catch (FileNotFoundException | NullPointerException e)
         {
            e.printStackTrace();
            System.err.println("Setting to default coeffs.");
         }
      }
      else
      {
         LogTools.info("Did not find: \"" + fingerTransmissionFile + "\", setting coeffs to default.");
      }
      return areCoeffsLoaded;
   }

   private void loadDefaultCoeffs(SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentScales,
                                  SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentBiases)
   {
      double defaultScaleFingerPitch1 = 0.40;
      double defaultScaleFingerPitch2 = 0.50;
      double defaultScaleFingerPitch3 = 0.30;

      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, YoDouble> scales = sideDependentScales.get(robotSide);
         EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiases.get(robotSide);

         scales.get(ThumbRoll).set(robotSide.negateIfLeftSide(1.0));
         scales.get(ThumbPitch1).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch1));
         scales.get(IndexFingerPitch1).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch1));
         scales.get(MiddleFingerPitch1).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch1));
         scales.get(PinkyPitch1).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch1));

         scales.get(ThumbPitch2).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch2));
         scales.get(IndexFingerPitch2).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch2));
         scales.get(MiddleFingerPitch2).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch2));
         scales.get(PinkyPitch2).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch2));

         scales.get(ThumbPitch3).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch3));
         scales.get(IndexFingerPitch3).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch3));
         scales.get(MiddleFingerPitch3).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch3));
         scales.get(PinkyPitch3).set(robotSide.negateIfLeftSide(defaultScaleFingerPitch3));
      }
   }

   private Predicate<DoubleProvider> createFingerJointPositionSwitchTrigger(RobotSide robotSide, ValkyrieHandJointName jointName, YoVariableRegistry registry)
   {
      YoBoolean isFingerJointEncoderDead = sideDependentEncoderDeadMap.get(robotSide).get(jointName);

      return new Predicate<DoubleProvider>()
      {
         private final int threshold = 100;
         private int deadCount = 0;
         private double lastValue = Double.NaN;

         @Override
         public boolean test(DoubleProvider encoderDataHolder)
         {
            if (forceMotorBasedPositionSwitch.getValue())
            {
               deadCount = 100;
            }
            else if (encoderDataHolder.getValue() == lastValue)
            {
               if (deadCount < threshold)
                  deadCount++;
            }
            else
            {
               deadCount = 0;
            }

            lastValue = encoderDataHolder.getValue();
            isFingerJointEncoderDead.set(deadCount >= threshold);
            return isFingerJointEncoderDead.getValue();
         }
      };
   }

   public void update()
   {
      RobotSide sideToCalibrateNow = doZeroFingerCalibrationNow.getValue();
      if (sideToCalibrateNow != null)
      {
         performZeroCalibrationNow(sideToCalibrateNow);
         startFingerCalibration.set(false);
         isCalibratingFingers.set(false);
         fingerCalibrationStartTime.setToNaN();
         doZeroFingerCalibrationNow.set(null);
      }

      performCalibrationUsingFingerJointEncoders();
   }

   private void performZeroCalibrationNow(RobotSide sideToCalibrateNow)
   {
      EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiases.get(sideToCalibrateNow);

      for (ValkyrieHandJointName jointName : ValkyrieHandJointName.values)
      {
         biases.get(jointName).set(0.0);
      }

      saveTransmissionCoeffsToFile(FINGER_TRANSMISSION_FILE);
   }

   private void performCalibrationUsingFingerJointEncoders()
   {
      double currentTime = Conversions.nanosecondsToSeconds(monotonicTimeProvider.getTimestamp());

      if (startFingerCalibration.getValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
            {
               sideDependentFingerPositionMeans.get(robotSide).get(jointEnum).clear();
            }
         }

         isCalibratingFingers.set(true);
         fingerCalibrationStartTime.set(currentTime);
         startFingerCalibration.set(false);
      }

      if (!isCalibratingFingers.getValue())
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, YoBoolean> encoderDeadMap = sideDependentEncoderDeadMap.get(robotSide);
         EnumMap<ValkyrieHandJointName, Mean> fingerPositionMeans = sideDependentFingerPositionMeans.get(robotSide);
         EnumMap<ValkyrieHandJointName, OneDoFJointBasics> fingerJoints = sideDependentFingerJoints.get(robotSide);

         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
         {
            if (encoderDeadMap.get(jointEnum).getBooleanValue())
               continue;

            fingerPositionMeans.get(jointEnum).increment(fingerJoints.get(jointEnum).getQ());
         }
      }

      if (currentTime >= fingerCalibrationDuration.getValue() + fingerCalibrationStartTime.getValue())
      {
         isCalibratingFingers.set(false);
         fingerCalibrationStartTime.setToNaN();

         for (RobotSide robotSide : RobotSide.values)
         {
            EnumMap<ValkyrieHandJointName, YoBoolean> encoderDeadMap = sideDependentEncoderDeadMap.get(robotSide);
            EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiases.get(robotSide);
            EnumMap<ValkyrieHandJointName, DoubleProvider> motorBasedFingerJointPositions = sideDependentMotorBasedFingerJointPositions.get(robotSide);
            EnumMap<ValkyrieHandJointName, Mean> fingerPositionMeans = sideDependentFingerPositionMeans.get(robotSide);

            for (ValkyrieHandJointName jointName : ValkyrieHandJointName.values)
            {
               if (encoderDeadMap.get(jointName).getBooleanValue())
                  continue;

               Mean mean = fingerPositionMeans.get(jointName);
               double offset = motorBasedFingerJointPositions.get(jointName).getValue() - mean.getResult();
               biases.get(jointName).sub(offset);
               mean.clear();
            }
         }

         saveTransmissionCoeffsToFile(FINGER_TRANSMISSION_FILE);
      }
   }

   private void saveTransmissionCoeffsToFile(String filePath)
   {
      Path transmissionFilePath = Paths.get(filePath);
      try
      {
         Files.createDirectories(transmissionFilePath.getParent());
         File file = transmissionFilePath.toFile();
         Yaml yaml = new Yaml();
         Map<String, Map<String, Double>> coeffs = new LinkedHashMap<>();
         for (RobotSide robotSide : RobotSide.values)
         {
            for (ValkyrieHandJointName jointName : ValkyrieHandJointName.values)
            {
               Map<String, Double> jointCoeffs = new LinkedHashMap<>();
               jointCoeffs.put("scale", sideDependentScales.get(robotSide).get(jointName).getDoubleValue());
               jointCoeffs.put("bias", sideDependentBiases.get(robotSide).get(jointName).getDoubleValue());
               coeffs.put(jointName.getJointName(robotSide), jointCoeffs);
            }
         }
         FileWriter output = new FileWriter(file);
         yaml.dump(coeffs, output);
         output.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void attachControllerAPI(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager)
   {
      statusOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, new StatusMessageListener<HighLevelStateChangeStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(HighLevelStateChangeStatusMessage statusMessage)
         {
            if (HighLevelControllerName.fromByte(statusMessage.getEndHighLevelControllerName()) == HighLevelControllerName.CALIBRATION)
               startFingerCalibration.set(true);
         }
      });
   }
   
   public double getFingerMotorPosition(RobotSide robotSide, ValkyrieFingerMotorName motorName)
   {
      return sideDependentFingerMotorHandles.get(robotSide).get(motorName).getQ();
   }

   public double getMotorBasedFingerJointPosition(RobotSide robotSide, ValkyrieHandJointName jointName)
   {
      return sideDependentMotorBasedFingerJointPositions.get(robotSide).get(jointName).getValue();
   }

   public double getFingerJointTransmissionScale(RobotSide robotSide, ValkyrieHandJointName jointName)
   {
      return sideDependentScales.get(robotSide).get(jointName).getValue();
   }

   @Override
   public SensorNoiseParameters getSensorNoiseParameters()
   {
      return sensorProcessingConfiguration.getSensorNoiseParameters();
   }

   @Override
   public double getEstimatorDT()
   {
      return sensorProcessingConfiguration.getEstimatorDT();
   }
}
