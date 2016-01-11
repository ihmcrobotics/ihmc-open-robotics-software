package us.ihmc.wholeBodyController.diagnostics;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.HashSet;
import java.util.Set;
import java.util.logging.Logger;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.IMUSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OrientationAngularVelocityConsistencyChecker;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.wholeBodyController.diagnostics.utils.DiagnosticTask;

public class PelvisIMUCheckUpDiagnosticTask extends DiagnosticTask
{
   private Logger logger;
   private final NumberFormat doubleFormat = new DecimalFormat("0.000;-0.000");

   private final YoVariableRegistry registry;

   private final IMUDefinition imuDefinition;

   private final IMUSensorValidityChecker validityChecker;
   private final OrientationAngularVelocityConsistencyChecker orientationVelocityConsistencyChecker;

   private final YoFunctionGenerator functionGenerator;
   private final DoubleYoVariable checkUpDuration;

   private final DoubleYoVariable rampDuration;
   private final DoubleYoVariable ramp;

   private final DiagnosticParameters diagnosticParameters;

   private final Set<OneDoFJoint> rollJointsAttachedToPelvis = new HashSet<>();
   private final OneDoFJoint spineRollJoint;
   private final SideDependentList<OneDoFJoint> hipRollJoints = new SideDependentList<>();
   private final DoubleYoVariable desiredJointPositionOffset;
   private final DoubleYoVariable desiredJointVelocityOffset;
   private final DoubleYoVariable desiredJointTauOffset;

   public PelvisIMUCheckUpDiagnosticTask(String imuToCheck, DiagnosticControllerToolbox toolbox, YoVariableRegistry parentRgistry)
   {
      this(findIMUDefinition(imuToCheck, toolbox), toolbox, parentRgistry);
   }

   public PelvisIMUCheckUpDiagnosticTask(IMUDefinition imuToCheck, DiagnosticControllerToolbox toolbox, YoVariableRegistry parentRgistry)
   {
      FullHumanoidRobotModel fullRobotModel = toolbox.getFullRobotModel();
      RigidBody pelvis = fullRobotModel.getPelvis();
      if (imuToCheck.getRigidBody() != pelvis)
         throw new RuntimeException("The IMU: " + imuToCheck.getName() + " is not attached to the pelvis, cannot create check up diagnostic for it.");

      this.imuDefinition = imuToCheck;
      String imuName = imuDefinition.getName();
      String nameSuffix = "CheckUp";

      registry = new YoVariableRegistry(imuName + nameSuffix);
      parentRgistry.addChild(registry);
      diagnosticParameters = toolbox.getDiagnosticParameters();

      desiredJointPositionOffset = new DoubleYoVariable("q_off_d_pelvis" + nameSuffix, registry);
      desiredJointVelocityOffset = new DoubleYoVariable("qd_off_d_pelvis" + nameSuffix, registry);
      desiredJointTauOffset = new DoubleYoVariable("tau_off_d_pelvis" + nameSuffix, registry);
      checkUpDuration = new DoubleYoVariable(imuName + nameSuffix + "Duration", registry);
      checkUpDuration.set(diagnosticParameters.getJointCheckUpDuration());

      rampDuration = new DoubleYoVariable(imuName + nameSuffix + "SignalRampDuration", registry);
      rampDuration.set(0.2 * checkUpDuration.getDoubleValue());
      ramp = new DoubleYoVariable(imuName + nameSuffix + "SignalRamp", registry);

      validityChecker = toolbox.getIMUSensorValidityChecker(imuToCheck);
      orientationVelocityConsistencyChecker = toolbox.getIMUOrientationAngularVelocityConsistencyChecker(imuToCheck);

      DoubleYoVariable yoTime = toolbox.getYoTime();
      functionGenerator = new YoFunctionGenerator(imuName + nameSuffix, yoTime, registry);
      functionGenerator.setAmplitude(diagnosticParameters.getCheckUpOscillationPositionAmplitude());
      functionGenerator.setFrequency(diagnosticParameters.getCheckUpOscillationPositionFrequency());
      functionGenerator.setResetTime(checkUpDuration.getDoubleValue());
      functionGenerator.setMode(YoFunctionGeneratorMode.SINE);

      spineRollJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      rollJointsAttachedToPelvis.add(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL));
      for (RobotSide robotSide : RobotSide.values)
      {
         rollJointsAttachedToPelvis.add(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_ROLL));
         hipRollJoints.put(robotSide, fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_ROLL));
      }
   }

   private static IMUDefinition findIMUDefinition(String imuName, DiagnosticControllerToolbox toolbox)
   {
      IMUDefinition imuDefinition = null;

      IMUDefinition[] imuDefinitions = toolbox.getFullRobotModel().getIMUDefinitions();

      for (int i = 0; i < imuDefinitions.length; i++)
      {
         if (imuDefinitions[i].getName().equals(imuName))
            imuDefinition = imuDefinitions[i];
      }

      if (imuDefinition == null)
         throw new RuntimeException("Could not find the IMUDefinition for the imu: " + imuName);

      return imuDefinition;
   }

   public void setupForLogging()
   {
      logger = Logger.getLogger(registry.getName());
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (logger != null)
         logger.info("Starting check up for the IMU: " + imuDefinition.getName());

      ramp.set(0.0);
   }

   private boolean enableEstimators = true;
   private boolean disableEstimators = true;

   @Override
   public void doAction()
   {
      if (getTimeInCurrentTask() < rampDuration.getDoubleValue())
         ramp.set(getTimeInCurrentTask() / rampDuration.getDoubleValue());
      else if (getTimeInCurrentTask() > checkUpDuration.getDoubleValue() + rampDuration.getDoubleValue())
      {
         if (disableEstimators)
         {
            reportCheckUpResults();
            orientationVelocityConsistencyChecker.disableAll();
            disableEstimators = false;
         }
         ramp.set(- (getTimeInCurrentTask() - checkUpDuration.getDoubleValue() - rampDuration.getDoubleValue()) / rampDuration.getDoubleValue());
      }
      else
      {
         if (enableEstimators)
         {
            orientationVelocityConsistencyChecker.enableAll();
            enableEstimators = false;
         }
         ramp.set(1.0);
      }

      ramp.set(MathTools.clipToMinMax(ramp.getDoubleValue(), 0.0, 1.0));

      double positionOffset = ramp.getDoubleValue() * functionGenerator.getValue(getTimeInCurrentTask());
      double velocityOffset = ramp.getDoubleValue() * functionGenerator.getValueDot();

      desiredJointPositionOffset.set(positionOffset);
      desiredJointVelocityOffset.set(velocityOffset);
      desiredJointTauOffset.set(0.0);

   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   private void reportCheckUpResults()
   {
      if (logger == null)
         return;

   }

   @Override
   public boolean isDone()
   {
      if (validityChecker.sensorsCannotBeTrusted())
      {
         if (logger != null)
            logger.severe(imuDefinition.getName() + " IMU sensor can't be trusted, skipping to the next diagnostic task.");
         return true;
      }

      return getTimeInCurrentTask() >= checkUpDuration.getDoubleValue() + 2.0 * rampDuration.getDoubleValue();
   }

   @Override
   public boolean abortRequested()
   {
      return false;
   }

   @Override
   public double getDesiredJointPositionOffset(OneDoFJoint joint)
   {
      if (rollJointsAttachedToPelvis.contains(joint))
         return desiredJointPositionOffset.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public double getDesiredJointVelocityOffset(OneDoFJoint joint)
   {
      if (rollJointsAttachedToPelvis.contains(joint))
         return desiredJointVelocityOffset.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public double getDesiredJointTauOffset(OneDoFJoint joint)
   {
      if (rollJointsAttachedToPelvis.contains(joint))
         return desiredJointTauOffset.getDoubleValue();
      else
         return 0.0;
   }
}
