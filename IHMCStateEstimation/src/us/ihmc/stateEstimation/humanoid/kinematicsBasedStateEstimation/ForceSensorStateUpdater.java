package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.subscribers.RequestWristForceSensorCalibrationSubscriber;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

public class ForceSensorStateUpdater implements ForceSensorCalibrationModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final ForceSensorDataHolderReadOnly inputForceSensorDataHolder;
   private final ForceSensorDataHolder forceSensorDataHolderToUpdate;
   private final ForceSensorDataHolder outputForceSensorDataHolder;
   private final ForceSensorDataHolder outputForceSensorDataHolderWithGravityCancelled;

   private final SideDependentList<ForceSensorDefinition> wristForceSensorDefinitions;
   private final SideDependentList<CenterOfMassReferenceFrame> wristsubtreeCenterOfMassFrames;

   private final BooleanYoVariable calibrateWristForceSensors;
   private final SideDependentList<DoubleYoVariable> wristSubtreeMass;
   private final SideDependentList<YoFrameVector> wristForcesSubtreeWeightCancelled;
   private final SideDependentList<YoFrameVector> wristTorquesSubtreeWeightCancelled;

   private final SideDependentList<YoFrameVector> wristForceCalibrationOffsets;
   private final SideDependentList<YoFrameVector> wristTorqueCalibrationOffsets;

   private final BooleanYoVariable calibrateFootForceSensors;
   private final AtomicBoolean calibrateFootForceSensorsAtomic = new AtomicBoolean(false);

   private final SideDependentList<ForceSensorDefinition> footForceSensorDefinitions;

   private final SideDependentList<YoFrameVector> footForceCalibrationOffsets;
   private final SideDependentList<YoFrameVector> footTorqueCalibrationOffsets;

   private RequestWristForceSensorCalibrationSubscriber requestWristForceSensorCalibrationSubscriber = null;

   private final Wrench wristWrenchDueToGravity = new Wrench();
   private final Wrench tempWrench = new Wrench();
   private final FrameVector tempForce = new FrameVector();
   private final FrameVector tempTorque = new FrameVector();

   private final double gravity;

   // For the viz
   private final Map<RigidBody, Wrench> wrenches;
   private final WrenchVisualizer wrenchVisualizer;

   private final boolean hasWristForceSensors;
   private final boolean hasFootForceSensors;

   public ForceSensorStateUpdater(SensorOutputMapReadOnly sensorOutputMapReadOnly, ForceSensorDataHolder forceSensorDataHolderToUpdate,
         StateEstimatorParameters stateEstimatorParameters, double gravity, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentreRegistry)
   {
      this.gravity = Math.abs(gravity);
      inputForceSensorDataHolder = sensorOutputMapReadOnly.getForceSensorProcessedOutputs();
      this.forceSensorDataHolderToUpdate = forceSensorDataHolderToUpdate;

      outputForceSensorDataHolder = new ForceSensorDataHolder(inputForceSensorDataHolder.getForceSensorDefinitions());
      outputForceSensorDataHolderWithGravityCancelled = new ForceSensorDataHolder(inputForceSensorDataHolder.getForceSensorDefinitions());

      registry = new YoVariableRegistry(getClass().getSimpleName());
      parentreRegistry.addChild(registry);

      hasWristForceSensors = checkIfWristForceSensorsExist(stateEstimatorParameters);
      hasFootForceSensors = checkIfFootForceSensorsExist(stateEstimatorParameters);

      if (hasFootForceSensors)
      {
         footForceSensorDefinitions = new SideDependentList<>();
         SideDependentList<String> footForceSensorNames = stateEstimatorParameters.getFootForceSensorNames();

         for (RobotSide robotSide : RobotSide.values)
         {
            ForceSensorDefinition forceSensorDefinition = inputForceSensorDataHolder.findForceSensorDefinition(footForceSensorNames.get(robotSide));
            footForceSensorDefinitions.put(robotSide, forceSensorDefinition);
         }

         footForceCalibrationOffsets = new SideDependentList<>();
         footTorqueCalibrationOffsets = new SideDependentList<>();

         calibrateFootForceSensors = new BooleanYoVariable("calibrateFootForceSensors", registry);
         calibrateFootForceSensors.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
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

            footForceCalibrationOffsets.put(robotSide, new YoFrameVector(namePrefix + "ForceCalibrationOffset", measurementFrame, registry));
            footTorqueCalibrationOffsets.put(robotSide, new YoFrameVector(namePrefix + "TorqueCalibrationOffset", measurementFrame, registry));
         }
      }
      else
      {
         footForceSensorDefinitions = null;
         footForceCalibrationOffsets = null;
         footTorqueCalibrationOffsets = null;
         calibrateFootForceSensors = null;
      }

      if (hasWristForceSensors)
      {
         wristForceSensorDefinitions = new SideDependentList<>();

         SideDependentList<String> wristForceSensorNames = stateEstimatorParameters.getWristForceSensorNames();

         for (RobotSide robotSide : RobotSide.values)
         {
            ForceSensorDefinition forceSensorDefinition = inputForceSensorDataHolder.findForceSensorDefinition(wristForceSensorNames.get(robotSide));
            wristForceSensorDefinitions.put(robotSide, forceSensorDefinition);
         }

         wristForcesSubtreeWeightCancelled = new SideDependentList<>();
         wristTorquesSubtreeWeightCancelled = new SideDependentList<>();
         wristForceCalibrationOffsets = new SideDependentList<>();
         wristTorqueCalibrationOffsets = new SideDependentList<>();
         wristSubtreeMass = new SideDependentList<>();
         wristsubtreeCenterOfMassFrames = new SideDependentList<>();

         calibrateWristForceSensors = new BooleanYoVariable("calibrateWristForceSensors", registry);
         calibrateWristForceSensors.set(stateEstimatorParameters.requestWristForceSensorCalibrationAtStart());

         for (RobotSide robotSide : RobotSide.values)
         {
            ForceSensorDefinition forceSensorDefinition = wristForceSensorDefinitions.get(robotSide);
            ReferenceFrame measurementFrame = forceSensorDefinition.getSensorFrame();
            RigidBody measurementLink = forceSensorDefinition.getRigidBody();

            String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
            String namePrefix = sidePrefix + "WristSensor";

            wristForcesSubtreeWeightCancelled.put(robotSide, new YoFrameVector(namePrefix + "ForceSubtreeWeightCancelled", measurementFrame, registry));
            wristTorquesSubtreeWeightCancelled.put(robotSide, new YoFrameVector(namePrefix + "TorqueSubtreeWeightCancelled", measurementFrame, registry));

            wristForceCalibrationOffsets.put(robotSide, new YoFrameVector(namePrefix + "ForceCalibrationOffset", measurementFrame, registry));
            wristTorqueCalibrationOffsets.put(robotSide, new YoFrameVector(namePrefix + "TorqueCalibrationOffset", measurementFrame, registry));

            RigidBody[] handBodies = ScrewTools.computeRigidBodiesAfterThisJoint(measurementLink.getParentJoint());
            CenterOfMassReferenceFrame subtreeCoMFrame = new CenterOfMassReferenceFrame(namePrefix + "SubtreeCoMFrame", measurementFrame, handBodies);
            wristsubtreeCenterOfMassFrames.put(robotSide, subtreeCoMFrame);
            DoubleYoVariable handMass = new DoubleYoVariable(namePrefix + "SubtreeMass", registry);
            wristSubtreeMass.put(robotSide, handMass);
            handMass.set(TotalMassCalculator.computeSubTreeMass(measurementLink));
         }

         if (yoGraphicsListRegistry == null)
         {
            wrenches = null;
            wrenchVisualizer = null;
         }
         else
         {
            wrenches = new LinkedHashMap<RigidBody, Wrench>();

            for (RobotSide robotSide : RobotSide.values)
            {
               ForceSensorDefinition forceSensorDefinition = inputForceSensorDataHolder.findForceSensorDefinition(wristForceSensorNames.get(robotSide));
               RigidBody measurementLink = forceSensorDefinition.getRigidBody();
               wrenches.put(measurementLink, new Wrench());
            }

            List<RigidBody> bodies = new ArrayList<>(wrenches.keySet());
            double forceVizScaling = 10.0;
            AppearanceDefinition forceAppearance = YoAppearance.DarkRed();
            AppearanceDefinition torqueAppearance = YoAppearance.DarkBlue();
            wrenchVisualizer = new WrenchVisualizer("ForceSensorData", bodies, forceVizScaling, yoGraphicsListRegistry, registry, forceAppearance,
                  torqueAppearance);
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
   }

   private boolean checkIfWristForceSensorsExist(StateEstimatorParameters stateEstimatorParameters)
   {
      SideDependentList<String> wristForceSensorNames = stateEstimatorParameters.getWristForceSensorNames();

      if (wristForceSensorNames == null || wristForceSensorNames.isEmpty())
      {
         return false;
      }

      // Make sure that both sensors actually exist
      if (inputForceSensorDataHolder.findForceSensorDefinition(wristForceSensorNames.get(RobotSide.LEFT)) == null
            || inputForceSensorDataHolder.findForceSensorDefinition(wristForceSensorNames.get(RobotSide.RIGHT)) == null)
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
      if (inputForceSensorDataHolder.findForceSensorDefinition(footForceSensorNames.get(RobotSide.LEFT)) == null
            || inputForceSensorDataHolder.findForceSensorDefinition(footForceSensorNames.get(RobotSide.RIGHT)) == null)
      {
         return false;
      }

      return true;
   }

   public void initialize()
   {
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
   }

   private void updateFootForceSensorState()
   {
      if (calibrateFootForceSensorsAtomic.getAndSet(false))
      {
         calibrateFootForceSensors.set(false);
         calibrateFootForceSensors();
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDefinition footForceSensorDefinition = footForceSensorDefinitions.get(robotSide);
         ForceSensorDataReadOnly footForceSensor = inputForceSensorDataHolder.get(footForceSensorDefinition);
         footForceSensor.getWrench(tempWrench);

         footForceCalibrationOffsets.get(robotSide).getFrameTupleIncludingFrame(tempForce);
         footTorqueCalibrationOffsets.get(robotSide).getFrameTupleIncludingFrame(tempTorque);

         tempWrench.subLinearPart(tempForce);
         tempWrench.subAngularPart(tempTorque);

         outputForceSensorDataHolder.setForceSensorValue(footForceSensorDefinition, tempWrench);
      }
   }

   private void updateWristForceSensorState()
   {
      if (requestWristForceSensorCalibrationSubscriber != null && requestWristForceSensorCalibrationSubscriber.checkForNewCalibrationRequest())
         calibrateWristForceSensors.set(true);

      if (calibrateWristForceSensors.getBooleanValue())
         calibrateWristForceSensors();

      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDefinition wristForceSensorDefinition = wristForceSensorDefinitions.get(robotSide);
         ForceSensorDataReadOnly wristForceSensor = inputForceSensorDataHolder.get(wristForceSensorDefinition);
         ReferenceFrame measurementFrame = wristForceSensor.getMeasurementFrame();
         RigidBody measurementLink = wristForceSensorDefinition.getRigidBody();
         wristForceSensor.getWrench(tempWrench);

         wristForceCalibrationOffsets.get(robotSide).getFrameTupleIncludingFrame(tempForce);
         wristTorqueCalibrationOffsets.get(robotSide).getFrameTupleIncludingFrame(tempTorque);

         tempWrench.subLinearPart(tempForce);
         tempWrench.subAngularPart(tempTorque);

         outputForceSensorDataHolder.setForceSensorValue(wristForceSensorDefinition, tempWrench);

         tempWrench.getLinearPartIncludingFrame(tempForce);
         tempWrench.getAngularPartIncludingFrame(tempTorque);

         cancelHandWeight(robotSide, tempWrench, measurementFrame);

         tempWrench.getLinearPartIncludingFrame(tempForce);
         tempWrench.getAngularPartIncludingFrame(tempTorque);

         wristForcesSubtreeWeightCancelled.get(robotSide).setAndMatchFrame(tempForce);
         wristTorquesSubtreeWeightCancelled.get(robotSide).setAndMatchFrame(tempTorque);

         if (wrenches != null)
            wrenches.get(measurementLink).set(tempWrench);

         outputForceSensorDataHolderWithGravityCancelled.setForceSensorValue(wristForceSensorDefinition, tempWrench);
      }
   }

   private void calibrateFootForceSensors()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDataReadOnly footForceSensor = inputForceSensorDataHolder.get(footForceSensorDefinitions.get(robotSide));
         footForceSensor.getWrench(tempWrench);

         tempWrench.getLinearPartIncludingFrame(tempForce);
         tempWrench.getAngularPartIncludingFrame(tempTorque);

         footForceCalibrationOffsets.get(robotSide).setAndMatchFrame(tempForce);
         footTorqueCalibrationOffsets.get(robotSide).setAndMatchFrame(tempTorque);
      }
      calibrateFootForceSensors.set(false);
   }

   private void calibrateWristForceSensors()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDataReadOnly wristForceSensor = inputForceSensorDataHolder.get(wristForceSensorDefinitions.get(robotSide));
         ReferenceFrame measurementFrame = wristForceSensor.getMeasurementFrame();

         wristForceSensor.getWrench(tempWrench);
         cancelHandWeight(robotSide, tempWrench, measurementFrame);

         tempWrench.getLinearPartIncludingFrame(tempForce);
         tempWrench.getAngularPartIncludingFrame(tempTorque);

         wristForceCalibrationOffsets.get(robotSide).setAndMatchFrame(tempForce);
         wristTorqueCalibrationOffsets.get(robotSide).setAndMatchFrame(tempTorque);
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
      wristWrenchDueToGravity.setLinearPart(tempForce);
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

   public void setRequestWristForceSensorCalibrationSubscriber(RequestWristForceSensorCalibrationSubscriber requestWristForceSensorCalibrationSubscriber)
   {
      this.requestWristForceSensorCalibrationSubscriber = requestWristForceSensorCalibrationSubscriber;
   }

   @Override
   public void requestFootForceSensorCalibrationAtomic()
   {
      calibrateFootForceSensorsAtomic.set(true);
   }
}
