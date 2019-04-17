package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.wholeBodyController.JointTorqueOffsetProcessor;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointTorqueOffsetEstimatorController implements RobotController, JointTorqueOffsetEstimator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private JointTorqueOffsetProcessor jointTorqueOffsetProcessor;

   private FullHumanoidRobotModel fullRobotModel;
   private final ArrayList<OneDoFJointBasics> oneDoFJoints = new ArrayList<OneDoFJointBasics>();

   private final LinkedHashMap<OneDoFJointBasics, PDController> pdControllers = new LinkedHashMap<OneDoFJointBasics, PDController>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> desiredPositions = new LinkedHashMap<OneDoFJointBasics, YoDouble>();
   private final LinkedHashMap<OneDoFJointBasics, DiagnosticsWhenHangingHelper> helpers = new LinkedHashMap<OneDoFJointBasics, DiagnosticsWhenHangingHelper>();

   private final YoDouble ditherAmplitude = new YoDouble("ditherAmplitude", registry);
   private final YoDouble ditherFrequency = new YoDouble("ditherFrequency", registry);

   private final YoDouble maximumTorqueOffset = new YoDouble("maximumTorqueOffset", registry);

   private final YoBoolean estimateTorqueOffset = new YoBoolean("estimateTorqueOffset", registry);
   private final YoBoolean transferTorqueOffsets = new YoBoolean("transferTorqueOffsets", registry);
   private final YoBoolean exportJointTorqueOffsetsToFile = new YoBoolean("recordTorqueOffsets", registry);

   private final boolean useArms = true;

   private final TorqueOffsetPrinter torqueOffsetPrinter;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<YoPlaneContactState> footContactStates;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final YoBoolean hasReachedMaximumTorqueOffset = new YoBoolean("hasReachedMaximumTorqueOffset", registry);

   private final YoDouble currentTime;

   public JointTorqueOffsetEstimatorController(WholeBodySetpointParameters wholeBodySetpointParameters,
                                               HighLevelHumanoidControllerToolbox highLevelControllerToolbox, TorqueOffsetPrinter torqueOffsetPrinter)
   {
      this.bipedSupportPolygons = highLevelControllerToolbox.getBipedSupportPolygons();
      this.footContactStates = highLevelControllerToolbox.getFootContactStates();
      this.controllerToolbox = highLevelControllerToolbox;
      this.torqueOffsetPrinter = torqueOffsetPrinter;
      this.fullRobotModel = highLevelControllerToolbox.getFullRobotModel();
      this.currentTime = highLevelControllerToolbox.getYoTime();

      ditherAmplitude.set(0.3);
      ditherFrequency.set(5.0);
      maximumTorqueOffset.set(5.0);

      estimateTorqueOffset.set(false);
      transferTorqueOffsets.set(false);

      fullRobotModel.getOneDoFJoints(oneDoFJoints);

      OneDoFJointBasics[] jointArray = fullRobotModel.getOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(jointArray);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(jointArray, JointDesiredControlMode.EFFORT);

      createHelpers(true);

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJointBasics joint = oneDoFJoints.get(i);

         String jointName = joint.getName();
         YoDouble desiredPosition = new YoDouble("q_d_calib_" + jointName, registry);
         desiredPosition.set(wholeBodySetpointParameters.getSetpoint(jointName));
         desiredPositions.put(joint, desiredPosition);

         if (!hasTorqueOffsetForJoint(joint))
            continue;

         PDController controller = new PDController(jointName + "Calibration", registry);
         pdControllers.put(joint, controller);

      }

      setDefaultPDControllerGains();
   }

   public void attachJointTorqueOffsetProcessor(JointTorqueOffsetProcessor jointTorqueOffsetProcessor)
   {
      this.jointTorqueOffsetProcessor = jointTorqueOffsetProcessor;
   }

   @Override
   public void initialize()
   {
      hasReachedMaximumTorqueOffset.set(false);
   }

   @Override
   public void doControl()
   {
      bipedSupportPolygons.updateUsingContactStates(footContactStates);
      controllerToolbox.update();

      updateDiagnosticsWhenHangingHelpers();
      updatePDControllers();

      if (transferTorqueOffsets.getBooleanValue())
      {
         transferTorqueOffsets.set(false);
         transferTorqueOffsetsToOutputWriter();
      }

      if (exportJointTorqueOffsetsToFile.getBooleanValue() && torqueOffsetPrinter != null)
      {
         exportJointTorqueOffsetsToFile.set(false);
         exportTorqueOffsets();
      }

      for (int jointIndex = 0; jointIndex < oneDoFJoints.size(); jointIndex++)
      {
         OneDoFJointBasics joint = oneDoFJoints.get(jointIndex);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(joint, joint.getTau());
      }

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJointBasics joint = oneDoFJoints.get(i);
         if (!hasTorqueOffsetForJoint(joint))
         {
            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(joint, 0.0);
            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition(joint, desiredPositions.get(joint).getValue());
            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointVelocity(joint, 0.0);
         }
      }
   }

   public void updateDiagnosticsWhenHangingHelpers()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoints.get(i));
         if (diagnosticsWhenHangingHelper != null)
            diagnosticsWhenHangingHelper.update();
      }
   }

   private void updatePDControllers()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJointBasics joint = oneDoFJoints.get(i);
         if (hasTorqueOffsetForJoint(joint))
            updatePDController(joint, currentTime.getDoubleValue());
      }
   }

   private void updatePDController(OneDoFJointBasics oneDoFJoint, double timeInCurrentState)
   {
      PDController pdController = pdControllers.get(oneDoFJoint);
      double desiredPosition = desiredPositions.get(oneDoFJoint).getDoubleValue();
      double desiredVelocity = 0.0;

      double tau = pdController.compute(oneDoFJoint.getQ(), desiredPosition, oneDoFJoint.getQd(), desiredVelocity);

      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
      if (diagnosticsWhenHangingHelper != null)
      {
         tau = diagnosticsWhenHangingHelper.getTorqueToApply(tau, estimateTorqueOffset.getBooleanValue(), maximumTorqueOffset.getDoubleValue());
         if (hasReachedMaximumTorqueOffset.getBooleanValue()
               && Math.abs(diagnosticsWhenHangingHelper.getTorqueOffset()) == maximumTorqueOffset.getDoubleValue())
         {
            PrintTools.warn(this, "Reached maximum torque for at least one joint.");
            hasReachedMaximumTorqueOffset.set(true);
         }
      }

      double ditherTorque = ditherAmplitude.getDoubleValue() * Math.sin(2.0 * Math.PI * ditherFrequency.getDoubleValue() * timeInCurrentState);
      oneDoFJoint.setTau(tau + ditherTorque);
   }

   private void createHelpers(boolean robotIsHanging)
   {
      if (useArms)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            makeArmJointHelper(robotSide, true, ArmJointName.SHOULDER_PITCH);
            makeArmJointHelper(robotSide, false, ArmJointName.SHOULDER_ROLL);
            makeArmJointHelper(robotSide, false, ArmJointName.SHOULDER_YAW);
            makeArmJointHelper(robotSide, true, ArmJointName.ELBOW_PITCH);
         }
      }

      SideDependentList<JointBasics> topLegJoints = new SideDependentList<JointBasics>();
      for (RobotSide robotSide : RobotSide.values)
      {
         topLegJoints.set(robotSide, fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW));
      }

      OneDoFJointBasics spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW);
      helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, false, robotIsHanging, topLegJoints, registry));

      spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);
      helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, true, robotIsHanging, topLegJoints, registry));

      spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, false, robotIsHanging, topLegJoints, registry));

      for (RobotSide robotSide : RobotSide.values)
      {
         makeLegJointHelper(robotSide, false, LegJointName.HIP_YAW);
         makeLegJointHelper(robotSide, true, LegJointName.HIP_PITCH);
         makeLegJointHelper(robotSide, false, LegJointName.HIP_ROLL);
         makeLegJointHelper(robotSide, true, LegJointName.KNEE_PITCH);
         makeLegJointHelper(robotSide, true, LegJointName.ANKLE_PITCH);
         makeLegJointHelper(robotSide, false, LegJointName.ANKLE_ROLL);
      }
   }

   private void makeArmJointHelper(RobotSide robotSide, boolean preserveY, ArmJointName armJointName)
   {
      OneDoFJointBasics armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
      helpers.put(armJoint, new DiagnosticsWhenHangingHelper(armJoint, preserveY, registry));
   }

   private void makeLegJointHelper(RobotSide robotSide, boolean preserveY, LegJointName legJointName)
   {
      OneDoFJointBasics legJoint = fullRobotModel.getLegJoint(robotSide, legJointName);
      helpers.put(legJoint, new DiagnosticsWhenHangingHelper(legJoint, preserveY, registry));
   }

   private void setDefaultPDControllerGains()
   {
      if (useArms)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            pdControllers.get(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_YAW)).setProportionalGain(30.0);
            pdControllers.get(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_YAW)).setDerivativeGain(3.0);

            pdControllers.get(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH)).setProportionalGain(50.0);
            pdControllers.get(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH)).setDerivativeGain(5.0);

            pdControllers.get(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL)).setProportionalGain(50.0);
            pdControllers.get(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL)).setDerivativeGain(5.0);

            pdControllers.get(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH)).setProportionalGain(40.0);
            pdControllers.get(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH)).setDerivativeGain(4.0);
         }
      }

      pdControllers.get(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW)).setProportionalGain(30.0);
      pdControllers.get(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW)).setDerivativeGain(2.0);

      pdControllers.get(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH)).setProportionalGain(150.0);
      pdControllers.get(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH)).setDerivativeGain(8.0);

      pdControllers.get(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL)).setProportionalGain(150.0);
      pdControllers.get(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL)).setDerivativeGain(8.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW)).setProportionalGain(30.0);
         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW)).setDerivativeGain(2.0);

         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH)).setProportionalGain(150.0);
         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH)).setDerivativeGain(7.5);

         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_ROLL)).setProportionalGain(165);
         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_ROLL)).setDerivativeGain(6.0);

         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH)).setProportionalGain(80.0);
         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH)).setDerivativeGain(3.0);

         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH)).setProportionalGain(20.0);
         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH)).setDerivativeGain(2.0);

         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL)).setProportionalGain(16.0);
         pdControllers.get(fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL)).setDerivativeGain(1.0);
      }
   }

   public void estimateTorqueOffset(boolean estimate)
   {
      estimateTorqueOffset.set(estimate);
   }

   public void exportTorqueOffsets()
   {
      torqueOffsetPrinter.printTorqueOffsets(this);
   }

   public void transferTorqueOffsetsToOutputWriter()
   {
      if (jointTorqueOffsetProcessor == null)
         return;

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         DiagnosticsWhenHangingHelper helper = helpers.get(oneDoFJoints.get(i));

         if (helper != null)
         {
            double torqueOffset = helper.getTorqueOffset();
            jointTorqueOffsetProcessor.subtractTorqueOffset(oneDoFJoints.get(i), torqueOffset);
            helper.setTorqueOffset(0.0);
         }
      }
   }

   public double getJointCalibrationPosition(OneDoFJointBasics joint)
   {
      YoDouble yoDesiredPosition = desiredPositions.get(joint);
      if (yoDesiredPosition != null)
         return yoDesiredPosition.getValue();
      else
         return 0.0;
   }

   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public List<OneDoFJointBasics> getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   @Override
   public double getEstimatedJointTorqueOffset(OneDoFJointBasics joint)
   {
      DiagnosticsWhenHangingHelper helper = helpers.get(joint);
      return helper == null ? 0.0 : helper.getTorqueOffset();
   }

   @Override
   public void resetEstimatedJointTorqueOffset(OneDoFJointBasics joint)
   {
      if (hasTorqueOffsetForJoint(joint))
         helpers.get(joint).setTorqueOffset(0.0);
   }

   @Override
   public boolean hasTorqueOffsetForJoint(OneDoFJointBasics joint)
   {
      return helpers.containsKey(joint);
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return "Controller for estimating the joint torque offsets. It is based on " + DiagnosticsWhenHangingControllerState.class.getSimpleName() + ".";
   }

}
