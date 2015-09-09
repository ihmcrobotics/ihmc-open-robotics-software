package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.communication.subscribers.RequestWristForceSensorCalibrationSubscriber;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.FrameVector;
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
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class ForceSensorStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final ForceSensorDataHolderReadOnly inputForceSensorDataHolder;
   private final ForceSensorDataHolder forceSensorDataHolderToUpdate;
   private final ForceSensorDataHolder outputForceSensorDataHolder;
   private final ForceSensorDataHolder outputForceSensorDataHolderWithGravityCancelled;
   private final SideDependentList<ForceSensorDefinition> wristForceSensorDefinitions;
   private final SideDependentList<CenterOfMassReferenceFrame> subtreeCenterOfMassFrames;

   private final BooleanYoVariable calibrateWristForceSensors;
   private final SideDependentList<DoubleYoVariable> subtreeMass;
   private final SideDependentList<YoFrameVector> wristForcesSubtreeWeightCancelled;
   private final SideDependentList<YoFrameVector> wristTorquesSubtreeWeightCancelled;

   private final SideDependentList<YoFrameVector> wristForceCalibrationOffsets;
   private final SideDependentList<YoFrameVector> wristTorqueCalibrationOffsets;

   private RequestWristForceSensorCalibrationSubscriber requestWristForceSensorCalibrationSubscriber = null;

   private final Wrench wristWrenchDueToGravity = new Wrench();
   private final Wrench wristTempWrench = new Wrench();
   private final FrameVector tempWristForce = new FrameVector();
   private final FrameVector tempWristTorque = new FrameVector();

   private final double gravity;

   // For the viz
   private final Map<RigidBody, Wrench> wrenches;
   private final WrenchVisualizer wrenchVisualizer;

   public ForceSensorStateUpdater(SensorOutputMapReadOnly sensorOutputMapReadOnly, ForceSensorDataHolder forceSensorDataHolderToUpdate,
         StateEstimatorParameters stateEstimatorParameters, double gravity, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentreRegistry)
   {
      this.gravity = Math.abs(gravity);
      inputForceSensorDataHolder = sensorOutputMapReadOnly.getForceSensorProcessedOutputs();
      this.forceSensorDataHolderToUpdate = forceSensorDataHolderToUpdate;

      SideDependentList<String> wristForceSensorNames = stateEstimatorParameters.getWristForceSensorNames();

      if (wristForceSensorNames == null || wristForceSensorNames.isEmpty())
      {
         wristForceSensorDefinitions = null;
      }
      // Make sure that both sensors actually exist
      else if (inputForceSensorDataHolder.findForceSensorDefinition(wristForceSensorNames.get(RobotSide.LEFT)) == null
            || inputForceSensorDataHolder.findForceSensorDefinition(wristForceSensorNames.get(RobotSide.RIGHT)) == null)
      {
         wristForceSensorDefinitions = null;
      }
      else
      {
         wristForceSensorDefinitions = new SideDependentList<>();

         for (RobotSide robotSide : RobotSide.values)
         {
            ForceSensorDefinition forceSensorDefinition = inputForceSensorDataHolder.findForceSensorDefinition(wristForceSensorNames.get(robotSide));
            wristForceSensorDefinitions.put(robotSide, forceSensorDefinition);
         }
      }
      
      
      if (wristForceSensorDefinitions == null)
      {
         outputForceSensorDataHolder = null;
         outputForceSensorDataHolderWithGravityCancelled = null;
         registry = null;
         wristForcesSubtreeWeightCancelled = null;
         wristTorquesSubtreeWeightCancelled = null;
         wristForceCalibrationOffsets = null;
         wristTorqueCalibrationOffsets = null;
         subtreeMass = null;
         subtreeCenterOfMassFrames = null;
         calibrateWristForceSensors = null;

         wrenches = null;
         wrenchVisualizer = null;
      }
      else
      {
         outputForceSensorDataHolder = new ForceSensorDataHolder(inputForceSensorDataHolder.getForceSensorDefinitions());
         outputForceSensorDataHolderWithGravityCancelled = new ForceSensorDataHolder(inputForceSensorDataHolder.getForceSensorDefinitions());

         registry = new YoVariableRegistry(getClass().getSimpleName());
         parentreRegistry.addChild(registry);

         wristForcesSubtreeWeightCancelled = new SideDependentList<>();
         wristTorquesSubtreeWeightCancelled = new SideDependentList<>();
         wristForceCalibrationOffsets = new SideDependentList<>();
         wristTorqueCalibrationOffsets = new SideDependentList<>();
         subtreeMass = new SideDependentList<>();
         subtreeCenterOfMassFrames = new SideDependentList<>();

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
            subtreeCenterOfMassFrames.put(robotSide, subtreeCoMFrame);
            DoubleYoVariable handMass = new DoubleYoVariable(namePrefix + "SubtreeMass", registry);
            subtreeMass.put(robotSide, handMass);
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
            wrenchVisualizer = new WrenchVisualizer("ForceSensorData", bodies, forceVizScaling, yoGraphicsListRegistry, registry, forceAppearance, torqueAppearance);
         }
      }
   }

   public void initialize()
   {
      updateForceSensorState();
   }

   public void updateForceSensorState()
   {
      if (outputForceSensorDataHolder == null)
      {
         if (forceSensorDataHolderToUpdate != null)
            forceSensorDataHolderToUpdate.set(inputForceSensorDataHolder);
         return;
      }

      outputForceSensorDataHolder.set(inputForceSensorDataHolder);
      outputForceSensorDataHolderWithGravityCancelled.set(inputForceSensorDataHolder);

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
         wristForceSensor.packWrench(wristTempWrench);

         wristForceCalibrationOffsets.get(robotSide).getFrameTupleIncludingFrame(tempWristForce);
         wristTorqueCalibrationOffsets.get(robotSide).getFrameTupleIncludingFrame(tempWristTorque);

         wristTempWrench.subLinearPart(tempWristForce);
         wristTempWrench.subAngularPart(tempWristTorque);

         outputForceSensorDataHolder.setForceSensorValue(wristForceSensorDefinition, wristTempWrench);

         wristTempWrench.packLinearPartIncludingFrame(tempWristForce);
         wristTempWrench.packAngularPartIncludingFrame(tempWristTorque);

         cancelHandWeight(robotSide, wristTempWrench, measurementFrame);

         wristTempWrench.packLinearPartIncludingFrame(tempWristForce);
         wristTempWrench.packAngularPartIncludingFrame(tempWristTorque);

         wristForcesSubtreeWeightCancelled.get(robotSide).setAndMatchFrame(tempWristForce);
         wristTorquesSubtreeWeightCancelled.get(robotSide).setAndMatchFrame(tempWristTorque);

         if (wrenches != null)
            wrenches.get(measurementLink).set(wristTempWrench);

         outputForceSensorDataHolderWithGravityCancelled.setForceSensorValue(wristForceSensorDefinition, wristTempWrench);
      }

      if (forceSensorDataHolderToUpdate != null)
         forceSensorDataHolderToUpdate.set(outputForceSensorDataHolder);

      if (wrenchVisualizer != null)
         wrenchVisualizer.visualize(wrenches);
   }

   private void calibrateWristForceSensors()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDataReadOnly wristForceSensor = inputForceSensorDataHolder.get(wristForceSensorDefinitions.get(robotSide));
         ReferenceFrame measurementFrame = wristForceSensor.getMeasurementFrame();

         wristForceSensor.packWrench(wristTempWrench);
         cancelHandWeight(robotSide, wristTempWrench, measurementFrame);

         wristTempWrench.packLinearPartIncludingFrame(tempWristForce);
         wristTempWrench.packAngularPartIncludingFrame(tempWristTorque);

         wristForceCalibrationOffsets.get(robotSide).setAndMatchFrame(tempWristForce);
         wristTorqueCalibrationOffsets.get(robotSide).setAndMatchFrame(tempWristTorque);
      }
      calibrateWristForceSensors.set(false);
   }

   private void cancelHandWeight(RobotSide robotSide, Wrench wrenchToSubstractHandWeightTo, ReferenceFrame measurementFrame)
   {
      CenterOfMassReferenceFrame handCoMFrame = subtreeCenterOfMassFrames.get(robotSide);
      handCoMFrame.update();
      tempWristForce.setIncludingFrame(worldFrame, 0.0, 0.0, -subtreeMass.get(robotSide).getDoubleValue() * gravity);
      tempWristForce.changeFrame(handCoMFrame);
      wristWrenchDueToGravity.setToZero(measurementFrame, handCoMFrame);
      wristWrenchDueToGravity.setLinearPart(tempWristForce);
      wristWrenchDueToGravity.changeFrame(measurementFrame);

      wrenchToSubstractHandWeightTo.sub(wristWrenchDueToGravity);
   }

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
}
