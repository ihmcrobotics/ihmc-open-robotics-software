package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class ForceSensorStateUpdater implements ForceSensorCalibrationModule, SCS2YoGraphicHolder
{
   private static final boolean DEBUG = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private final ForceSensorDataHolderReadOnly inputForceSensorDataHolder;
   private final ForceSensorDataHolder forceSensorDataHolderToUpdate;
   private final ForceSensorDataHolder outputForceSensorDataHolder;
   private final ForceSensorDataHolder outputForceSensorDataHolderWithGravityCancelled;
   private final List<YoFrameVector3D> outputForces;

   private final SideDependentList<ForceSensorDefinition> wristForceSensorDefinitions;
   private final SideDependentList<CenterOfMassReferenceFrame> wristsubtreeCenterOfMassFrames;

   private final YoBoolean calibrateWristForceSensors;
   private final AtomicBoolean calibrateWristForceSensorsAtomic = new AtomicBoolean(false);
   private final SideDependentList<YoDouble> wristSubtreeMass;
   private final SideDependentList<YoFrameVector3D> wristForcesSubtreeWeightCancelled;
   private final SideDependentList<YoFrameVector3D> wristTorquesSubtreeWeightCancelled;

   private final SideDependentList<YoFrameVector3D> wristForceCalibrationOffsets;
   private final SideDependentList<YoFrameVector3D> wristTorqueCalibrationOffsets;

   private final YoBoolean calibrateFootForceSensors;
   private final AtomicBoolean calibrateFootForceSensorsAtomic = new AtomicBoolean(false);

   private final SideDependentList<ForceSensorDefinition> footForceSensorDefinitions;
   private final JointTorqueBasedWrenchSensorDriftEstimator footWrenchSensorDriftEstimator;

   private final SideDependentList<YoFrameVector3D> footForceCalibrationOffsets;
   private final SideDependentList<YoFrameVector3D> footTorqueCalibrationOffsets;

   private final Wrench wristWrenchDueToGravity = new Wrench();
   private final Wrench tempWrench = new Wrench();
   private final FrameVector3D tempForce = new FrameVector3D();
   private final FrameVector3D tempTorque = new FrameVector3D();

   private final double gravity;

   // For the viz
   private final Map<RigidBodyBasics, Wrench> wrenches;
   private final WrenchVisualizer wrenchVisualizer;

   private final boolean hasWristForceSensors;
   private final boolean hasFootForceSensors;

   public ForceSensorStateUpdater(FloatingJointBasics rootJoint,
                                  SensorOutputMapReadOnly sensorOutputMapReadOnly,
                                  ForceSensorDataHolder forceSensorDataHolderToUpdate,
                                  StateEstimatorParameters stateEstimatorParameters,
                                  double gravity,
                                  RobotMotionStatusHolder robotMotionStatusHolder,
                                  YoGraphicsListRegistry yoGraphicsListRegistry,
                                  YoRegistry parentreRegistry)
   {
      this.gravity = Math.abs(gravity);
      inputForceSensorDataHolder = sensorOutputMapReadOnly.getForceSensorOutputs();
      this.forceSensorDataHolderToUpdate = forceSensorDataHolderToUpdate;

      outputForceSensorDataHolder = new ForceSensorDataHolder(inputForceSensorDataHolder.getForceSensorDefinitions());
      outputForceSensorDataHolderWithGravityCancelled = new ForceSensorDataHolder(inputForceSensorDataHolder.getForceSensorDefinitions());

      registry = new YoRegistry(getClass().getSimpleName());
      parentreRegistry.addChild(registry);

      hasWristForceSensors = checkIfWristForceSensorsExist(stateEstimatorParameters);
      hasFootForceSensors = checkIfFootForceSensorsExist(stateEstimatorParameters);

      if (hasFootForceSensors)
      {
         footForceSensorDefinitions = new SideDependentList<>();
         SideDependentList<String> footForceSensorNames = stateEstimatorParameters.getFootForceSensorNames();

         for (RobotSide robotSide : RobotSide.values)
         {
            ForceSensorDefinition forceSensorDefinition = inputForceSensorDataHolder.getDefinition(footForceSensorNames.get(robotSide));
            footForceSensorDefinitions.put(robotSide, forceSensorDefinition);
         }

         footForceCalibrationOffsets = new SideDependentList<>();
         footTorqueCalibrationOffsets = new SideDependentList<>();

         calibrateFootForceSensors = new YoBoolean("calibrateFootForceSensors", registry);
         calibrateFootForceSensors.addListener(new YoVariableChangedListener()
         {
            @Override
            public void changed(YoVariable v)
            {
               if (calibrateFootForceSensors.getBooleanValue())
                  calibrateFootForceSensorsAtomic.set(true);
            }
         });

         calibrateFootForceSensors.set(stateEstimatorParameters.requestFootForceSensorCalibrationAtStart());

         for (RobotSide robotSide : RobotSide.values)
         {
            ForceSensorDefinition forceSensorDefinition = footForceSensorDefinitions.get(robotSide);
            ReferenceFrame measurementFrame = forceSensorDefinition.getSensorFrame();

            String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
            String namePrefix = sidePrefix + "FootSensor";

            footForceCalibrationOffsets.put(robotSide, new YoFrameVector3D(namePrefix + "ForceCalibrationOffset", measurementFrame, registry));
            footTorqueCalibrationOffsets.put(robotSide, new YoFrameVector3D(namePrefix + "TorqueCalibrationOffset", measurementFrame, registry));
         }

         if (stateEstimatorParameters.createFootWrenchSensorDriftEstimator())
         {
            double updateDT = stateEstimatorParameters.getEstimatorDT();

            footWrenchSensorDriftEstimator = new JointTorqueBasedWrenchSensorDriftEstimator(rootJoint,
                                                                                            updateDT,
                                                                                            robotMotionStatusHolder,
                                                                                            footForceSensorDefinitions.values(),
                                                                                            inputForceSensorDataHolder,
                                                                                            registry);
         }
         else
         {
            footWrenchSensorDriftEstimator = null;
         }
      }
      else
      {
         footForceSensorDefinitions = null;
         footForceCalibrationOffsets = null;
         footTorqueCalibrationOffsets = null;
         calibrateFootForceSensors = null;
         footWrenchSensorDriftEstimator = null;
      }

      if (hasWristForceSensors)
      {
         wristForceSensorDefinitions = new SideDependentList<>();

         SideDependentList<String> wristForceSensorNames = stateEstimatorParameters.getWristForceSensorNames();

         for (RobotSide robotSide : RobotSide.values)
         {
            ForceSensorDefinition forceSensorDefinition = inputForceSensorDataHolder.getDefinition(wristForceSensorNames.get(robotSide));
            wristForceSensorDefinitions.put(robotSide, forceSensorDefinition);
         }

         wristForcesSubtreeWeightCancelled = new SideDependentList<>();
         wristTorquesSubtreeWeightCancelled = new SideDependentList<>();
         wristForceCalibrationOffsets = new SideDependentList<>();
         wristTorqueCalibrationOffsets = new SideDependentList<>();
         wristSubtreeMass = new SideDependentList<>();
         wristsubtreeCenterOfMassFrames = new SideDependentList<>();

         calibrateWristForceSensors = new YoBoolean("calibrateWristForceSensors", registry);
         calibrateWristForceSensors.set(stateEstimatorParameters.requestWristForceSensorCalibrationAtStart());

         for (RobotSide robotSide : RobotSide.values)
         {
            ForceSensorDefinition forceSensorDefinition = wristForceSensorDefinitions.get(robotSide);
            ReferenceFrame measurementFrame = forceSensorDefinition.getSensorFrame();
            RigidBodyBasics measurementLink = forceSensorDefinition.getRigidBody();

            String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
            String namePrefix = sidePrefix + "WristSensor";

            wristForcesSubtreeWeightCancelled.put(robotSide, new YoFrameVector3D(namePrefix + "ForceSubtreeWeightCancelled", measurementFrame, registry));
            wristTorquesSubtreeWeightCancelled.put(robotSide, new YoFrameVector3D(namePrefix + "TorqueSubtreeWeightCancelled", measurementFrame, registry));

            wristForceCalibrationOffsets.put(robotSide, new YoFrameVector3D(namePrefix + "ForceCalibrationOffset", measurementFrame, registry));
            wristTorqueCalibrationOffsets.put(robotSide, new YoFrameVector3D(namePrefix + "TorqueCalibrationOffset", measurementFrame, registry));

            CenterOfMassReferenceFrame subtreeCoMFrame = new CenterOfMassReferenceFrame(namePrefix + "SubtreeCoMFrame", measurementFrame, measurementLink);
            wristsubtreeCenterOfMassFrames.put(robotSide, subtreeCoMFrame);
            YoDouble handMass = new YoDouble(namePrefix + "SubtreeMass", registry);
            wristSubtreeMass.put(robotSide, handMass);
            handMass.set(MultiBodySystemMissingTools.computeSubTreeMass(measurementLink));
         }

         if (yoGraphicsListRegistry == null)
         {
            wrenches = null;
            wrenchVisualizer = null;
         }
         else
         {
            wrenches = new LinkedHashMap<RigidBodyBasics, Wrench>();

            for (RobotSide robotSide : RobotSide.values)
            {
               ForceSensorDefinition forceSensorDefinition = inputForceSensorDataHolder.getDefinition(wristForceSensorNames.get(robotSide));
               RigidBodyBasics measurementLink = forceSensorDefinition.getRigidBody();
               wrenches.put(measurementLink, new Wrench());
            }

            List<RigidBodyBasics> bodies = new ArrayList<>(wrenches.keySet());
            double forceVizScaling = 10.0;
            AppearanceDefinition forceAppearance = YoAppearance.DarkRed();
            AppearanceDefinition torqueAppearance = YoAppearance.DarkBlue();
            wrenchVisualizer = new WrenchVisualizer("ForceSensorData", forceVizScaling, yoGraphicsListRegistry, registry, forceAppearance, torqueAppearance);
            wrenchVisualizer.registerRigidBodies(bodies);
         }
      }
      else
      {
         wristForceSensorDefinitions = null;
         wristForcesSubtreeWeightCancelled = null;
         wristTorquesSubtreeWeightCancelled = null;
         wristForceCalibrationOffsets = null;
         wristTorqueCalibrationOffsets = null;
         wristSubtreeMass = null;
         wristsubtreeCenterOfMassFrames = null;
         calibrateWristForceSensors = null;

         wrenches = null;
         wrenchVisualizer = null;
      }

      if (DEBUG)
      {
         outputForces = new ArrayList<>();
         for (int i = 0; i < outputForceSensorDataHolder.getNumberOfForceSensors(); i++)
         {
            ForceSensorDefinition definition = outputForceSensorDataHolder.getForceSensorDefinitions().get(i);
            outputForces.add(new YoFrameVector3D(definition.getSensorName() + "OutputForce", definition.getSensorFrame(), registry));
         }
      }
      else
      {
         outputForces = null;
      }
   }

   private boolean checkIfWristForceSensorsExist(StateEstimatorParameters stateEstimatorParameters)
   {
      SideDependentList<String> wristForceSensorNames = stateEstimatorParameters.getWristForceSensorNames();

      if (wristForceSensorNames == null || wristForceSensorNames.isEmpty())
      {
         return false;
      }

      // Make sure that both sensors actually exist
      if (inputForceSensorDataHolder.getDefinition(wristForceSensorNames.get(RobotSide.LEFT)) == null
          || inputForceSensorDataHolder.getDefinition(wristForceSensorNames.get(RobotSide.RIGHT)) == null)
      {
         return false;
      }

      return true;
   }

   private boolean checkIfFootForceSensorsExist(StateEstimatorParameters stateEstimatorParameters)
   {
      SideDependentList<String> footForceSensorNames = stateEstimatorParameters.getFootForceSensorNames();

      if (footForceSensorNames == null || footForceSensorNames.isEmpty())
      {
         return false;
      }

      // Make sure that both sensors actually exist
      if (inputForceSensorDataHolder.getDefinition(footForceSensorNames.get(RobotSide.LEFT)) == null
          || inputForceSensorDataHolder.getDefinition(footForceSensorNames.get(RobotSide.RIGHT)) == null)
      {
         return false;
      }

      return true;
   }

   public void initialize()
   {
      if (footWrenchSensorDriftEstimator != null)
         footWrenchSensorDriftEstimator.reset();

      updateForceSensorState();
   }

   public void updateForceSensorState()
   {
      outputForceSensorDataHolder.set(inputForceSensorDataHolder);
      outputForceSensorDataHolderWithGravityCancelled.set(inputForceSensorDataHolder);

      if (hasFootForceSensors)
         updateFootForceSensorState();

      if (hasWristForceSensors)
         updateWristForceSensorState();

      if (forceSensorDataHolderToUpdate != null)
         forceSensorDataHolderToUpdate.set(outputForceSensorDataHolder);

      if (wrenchVisualizer != null)
         wrenchVisualizer.visualize(wrenches);

      if (DEBUG)
      {
         for (int i = 0; i < outputForceSensorDataHolder.getNumberOfForceSensors(); i++)
         {
            outputForces.get(i).set(outputForceSensorDataHolder.getForceSensorDatas().get(i).getWrench().getLinearPart());
         }
      }
   }

   private void updateFootForceSensorState()
   {
      if (calibrateFootForceSensorsAtomic.getAndSet(false))
      {
         calibrateFootForceSensors.set(false);
         calibrateFootForceSensors();
         if (footWrenchSensorDriftEstimator != null)
            footWrenchSensorDriftEstimator.reset();
      }

      if (footWrenchSensorDriftEstimator != null)
         footWrenchSensorDriftEstimator.update();

      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDefinition footForceSensorDefinition = footForceSensorDefinitions.get(robotSide);
         ForceSensorDataReadOnly footForceSensor = inputForceSensorDataHolder.getData(footForceSensorDefinition);

         tempWrench.setIncludingFrame(footForceSensor.getWrench());
         tempWrench.getLinearPart().sub(footForceCalibrationOffsets.get(robotSide));
         tempWrench.getAngularPart().sub(footTorqueCalibrationOffsets.get(robotSide));

         if (footWrenchSensorDriftEstimator != null)
            tempWrench.sub(footWrenchSensorDriftEstimator.getSensortDriftBias(footForceSensorDefinition));

         outputForceSensorDataHolder.getData(footForceSensorDefinition).setWrench(tempWrench);
      }
   }

   private void updateWristForceSensorState()
   {
      if (calibrateWristForceSensorsAtomic.getAndSet(false))
         calibrateWristForceSensors.set(true);

      if (calibrateWristForceSensors.getBooleanValue())
         calibrateWristForceSensors();

      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDefinition wristForceSensorDefinition = wristForceSensorDefinitions.get(robotSide);
         ForceSensorDataReadOnly wristForceSensor = inputForceSensorDataHolder.getData(wristForceSensorDefinition);
         ReferenceFrame measurementFrame = wristForceSensor.getMeasurementFrame();
         RigidBodyBasics measurementLink = wristForceSensorDefinition.getRigidBody();
         tempWrench.setIncludingFrame(wristForceSensor.getWrench());

         tempForce.setIncludingFrame(wristForceCalibrationOffsets.get(robotSide));
         tempTorque.setIncludingFrame(wristTorqueCalibrationOffsets.get(robotSide));

         tempWrench.getLinearPart().sub(tempForce);
         tempWrench.getAngularPart().sub(tempTorque);

         outputForceSensorDataHolder.getData(wristForceSensorDefinition).setWrench(tempWrench);

         tempForce.setIncludingFrame(tempWrench.getLinearPart());
         tempTorque.setIncludingFrame(tempWrench.getAngularPart());

         cancelHandWeight(robotSide, tempWrench, measurementFrame);

         tempForce.setIncludingFrame(tempWrench.getLinearPart());
         tempTorque.setIncludingFrame(tempWrench.getAngularPart());

         wristForcesSubtreeWeightCancelled.get(robotSide).setMatchingFrame(tempForce);
         wristTorquesSubtreeWeightCancelled.get(robotSide).setMatchingFrame(tempTorque);

         if (wrenches != null)
            wrenches.get(measurementLink).setIncludingFrame(tempWrench);

         outputForceSensorDataHolderWithGravityCancelled.getData(wristForceSensorDefinition).setWrench(tempWrench);
      }
   }

   private void calibrateFootForceSensors()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDataReadOnly footForceSensor = inputForceSensorDataHolder.getData(footForceSensorDefinitions.get(robotSide));
         tempForce.setIncludingFrame(footForceSensor.getWrench().getLinearPart());
         tempTorque.setIncludingFrame(footForceSensor.getWrench().getAngularPart());

         footForceCalibrationOffsets.get(robotSide).setMatchingFrame(tempForce);
         footTorqueCalibrationOffsets.get(robotSide).setMatchingFrame(tempTorque);
      }
      calibrateFootForceSensors.set(false);
   }

   private void calibrateWristForceSensors()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDataReadOnly wristForceSensor = inputForceSensorDataHolder.getData(wristForceSensorDefinitions.get(robotSide));
         ReferenceFrame measurementFrame = wristForceSensor.getMeasurementFrame();

         tempWrench.setIncludingFrame(wristForceSensor.getWrench());
         cancelHandWeight(robotSide, tempWrench, measurementFrame);

         tempForce.setIncludingFrame(tempWrench.getLinearPart());
         tempTorque.setIncludingFrame(tempWrench.getAngularPart());

         wristForceCalibrationOffsets.get(robotSide).setMatchingFrame(tempForce);
         wristTorqueCalibrationOffsets.get(robotSide).setMatchingFrame(tempTorque);
      }
      calibrateWristForceSensors.set(false);
   }

   private void cancelHandWeight(RobotSide robotSide, Wrench wrenchToSubstractHandWeightTo, ReferenceFrame measurementFrame)
   {
      CenterOfMassReferenceFrame handCoMFrame = wristsubtreeCenterOfMassFrames.get(robotSide);
      handCoMFrame.update();
      tempForce.setIncludingFrame(worldFrame, 0.0, 0.0, -wristSubtreeMass.get(robotSide).getDoubleValue() * gravity);
      tempForce.changeFrame(handCoMFrame);
      wristWrenchDueToGravity.setToZero(measurementFrame, handCoMFrame);
      wristWrenchDueToGravity.getLinearPart().set(tempForce);
      wristWrenchDueToGravity.changeFrame(measurementFrame);

      wrenchToSubstractHandWeightTo.sub(wristWrenchDueToGravity);
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorOutput()
   {
      if (outputForceSensorDataHolder == null)
         return inputForceSensorDataHolder;
      else
         return outputForceSensorDataHolder;
   }

   public ForceSensorDataHolder getForceSensorOutputWithGravityCancelled()
   {
      return outputForceSensorDataHolderWithGravityCancelled;
   }

   @Override
   public void requestFootForceSensorCalibrationAtomic()
   {
      calibrateFootForceSensorsAtomic.set(true);
   }

   public void requestWristForceSensorCalibrationAtomic()
   {
      calibrateWristForceSensorsAtomic.set(true);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      if (wrenchVisualizer != null)
         group.addChild(wrenchVisualizer.getSCS2YoGraphics());
      return group.isEmpty() ? null : group;
   }
}
