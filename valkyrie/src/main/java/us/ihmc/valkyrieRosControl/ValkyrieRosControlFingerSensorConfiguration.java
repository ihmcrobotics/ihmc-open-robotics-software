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
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

import org.apache.commons.lang.StringUtils;
import org.yaml.snakeyaml.Yaml;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName;
import us.ihmc.valkyrie.fingers.ValkyrieHandJointName;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoJointStateHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieRosControlFingerSensorConfiguration implements SensorProcessingConfiguration
{
   private static final String FINGER_TRANSMISSION_FILE = System.getProperty("user.home") + File.separator + "valkyrie/ValkyrieFingerJointTransmissionCoeffs.yaml";

   private final SensorProcessingConfiguration sensorProcessingConfiguration;
   private final SideDependentList<EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder>> sideDependentFingerMotorHandles = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);

   public ValkyrieRosControlFingerSensorConfiguration(List<YoEffortJointHandleHolder> yoEffortJointHandleHolders,
                                                      List<YoPositionJointHandleHolder> yoPositionJointHandleHolders,
                                                      List<YoJointStateHandleHolder> yoJointStateHandleHolders,
                                                      SensorProcessingConfiguration sensorProcessingConfiguration)
   {
      this.sensorProcessingConfiguration = sensorProcessingConfiguration;

      for (RobotSide robotSide : RobotSide.values)
      {
         for (ValkyrieFingerMotorName motor : ValkyrieFingerMotorName.values)
         {
            YoEffortJointHandleHolder handle = yoEffortJointHandleHolders.stream().filter(h -> h.getName().equals(motor.getJointName(robotSide))).findFirst()
                                                                         .get();
            sideDependentFingerMotorHandles.get(robotSide).put(motor, handle);
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

      SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentScales = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
      SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentBiases = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);

      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, YoDouble> scales = sideDependentScales.get(robotSide);
         EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiases.get(robotSide);

         for (ValkyrieHandJointName fingerJoint : ValkyrieHandJointName.values)
         {
            YoDouble scale = new YoDouble("scale" + fingerJoint.getPascalCaseJointName(robotSide), registry);
            YoDouble bias = new YoDouble("bias" + fingerJoint.getPascalCaseJointName(robotSide), registry);
            scales.put(fingerJoint, scale);
            biases.put(fingerJoint, bias);
         }
      }

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
               Predicate<DoubleProvider> backupTrigger = createFingerJointPositionSwitchTrigger(slaveJointName, registry);
               sensorProcessing.addJointPositionSensorSwitch(slaveJointName, motorBasedPosition, backupTrigger, false);
            }
         }

         ValkyrieFingerMotorName[] motors = {IndexFingerMotorPitch1, MiddleFingerMotorPitch1, PinkyMotorPitch1};
         ValkyrieHandJointName[][] slaveJoints = {
               {IndexFingerPitch1, MiddleFingerPitch1, PinkyPitch1},
               {IndexFingerPitch2, MiddleFingerPitch2, PinkyPitch2},
               {IndexFingerPitch3, MiddleFingerPitch3, PinkyPitch3}};

         for (int fingerIndex = 0; fingerIndex < 3; fingerIndex++)
         {
            YoEffortJointHandleHolder motorHandle = motorHandles.get(motors[fingerIndex]);

            for (int i = 0; i < slaveJoints.length; i++)
            {
               ValkyrieHandJointName slaveJoint = slaveJoints[i][fingerIndex];
               YoDouble scale = sideDependentScales.get(robotSide).get(slaveJoint);
               YoDouble bias = sideDependentBiases.get(robotSide).get(slaveJoint);
               String slaveJointName = slaveJoint.getCamelCaseJointName(robotSide);
               DoubleProvider motorBasedPosition = sensorProcessing.computeJointPositionUsingCoupling(motorHandle::getQ, slaveJointName, scale, bias, true);
               Predicate<DoubleProvider> backupTrigger = createFingerJointPositionSwitchTrigger(slaveJointName, registry);
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
         PrintTools.info(this, "Did not find: \"" + fingerTransmissionFile + "\", setting coeffs to default.");
      }
      return areCoeffsLoaded;
   }

   private void loadDefaultCoeffs(SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentScales,
                                  SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentBiases)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, YoDouble> scales = sideDependentScales.get(robotSide);
         EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiases.get(robotSide);

         boolean isLeftSide = robotSide == RobotSide.LEFT;

         scales.get(ThumbRoll         ).set(isLeftSide ? -1.00 : 1.00);
         scales.get(ThumbPitch1       ).set(isLeftSide ? -1.00 : 1.00);
         scales.get(IndexFingerPitch1 ).set(isLeftSide ? -0.70 : 0.70);
         scales.get(MiddleFingerPitch1).set(isLeftSide ? -0.70 : 0.70);
         scales.get(PinkyPitch1       ).set(isLeftSide ? -0.70 : 0.70);

         scales.get(ThumbPitch2       ).set(isLeftSide ? -0.60 : 0.60);
         scales.get(IndexFingerPitch2 ).set(isLeftSide ? 0.60 : 0.60);
         scales.get(MiddleFingerPitch2).set(isLeftSide ? 0.60 : 0.60);
         scales.get(PinkyPitch2       ).set(isLeftSide ? 0.60 : 0.60);

         scales.get(ThumbPitch3       ).set(isLeftSide ? 0.30 : 0.30);
         scales.get(IndexFingerPitch3 ).set(isLeftSide ? 0.30 : 0.30);
         scales.get(MiddleFingerPitch3).set(isLeftSide ? 0.30 : 0.30);
         scales.get(PinkyPitch3       ).set(isLeftSide ? 0.30 : 0.30);

         biases.get(ThumbRoll         ).set(1.57); // TODO at the same I added these, the thumb roll did not work.
         biases.get(ThumbPitch1       ).set(isLeftSide ? 0.00 : 0.00);
         biases.get(IndexFingerPitch1 ).set(isLeftSide ? 0.00 : 0.00);
         biases.get(MiddleFingerPitch1).set(isLeftSide ? 0.00 : 0.00);
         biases.get(PinkyPitch1       ).set(isLeftSide ? 0.00 : 0.00);

         biases.get(ThumbPitch2       ).set(isLeftSide ? 0.00 : 0.00);
         biases.get(IndexFingerPitch2 ).set(isLeftSide ? 0.00 : 0.00);
         biases.get(MiddleFingerPitch2).set(isLeftSide ? 0.00 : 0.00);
         biases.get(PinkyPitch2       ).set(isLeftSide ? 0.00 : 0.00);

         biases.get(ThumbPitch3       ).set(isLeftSide ? 0.00 : 0.00);
         biases.get(IndexFingerPitch3 ).set(isLeftSide ? 0.00 : 0.00);
         biases.get(MiddleFingerPitch3).set(isLeftSide ? 0.00 : 0.00);
         biases.get(PinkyPitch3       ).set(isLeftSide ? 0.00 : 0.00);
      }
   }

   private Predicate<DoubleProvider> createFingerJointPositionSwitchTrigger(String jointName, YoVariableRegistry registry)
   {
      YoBoolean isFingerJointEncoderDead = new YoBoolean("is" + StringUtils.capitalize(jointName) + "EncoderDead", registry);

      return new Predicate<DoubleProvider>()
      {
         private final int threshold = 100;
         private int deadCount = 0;

         @Override
         public boolean test(DoubleProvider encoderDataHolder)
         {
            if (encoderDataHolder.getValue() == 0.0)
            {
               if (deadCount < threshold)
                  deadCount++;
            }
            else
            {
               deadCount = 0;
            }

            isFingerJointEncoderDead.set(deadCount >= threshold);
            return isFingerJointEncoderDead.getValue();
         }
      };
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
