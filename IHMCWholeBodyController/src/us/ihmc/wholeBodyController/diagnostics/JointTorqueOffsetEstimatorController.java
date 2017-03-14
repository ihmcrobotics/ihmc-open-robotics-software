package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.JointTorqueOffsetProcessor;

public class JointTorqueOffsetEstimatorController extends HighLevelBehavior implements RobotController, JointTorqueOffsetEstimator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private JointTorqueOffsetProcessor jointTorqueOffsetProcessor;

   private FullHumanoidRobotModel fullRobotModel;
   private final ArrayList<OneDoFJoint> oneDoFJoints = new ArrayList<OneDoFJoint>();

   private final LinkedHashMap<OneDoFJoint, PDController> pdControllers = new LinkedHashMap<OneDoFJoint, PDController>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredPositions = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
   private final LinkedHashMap<OneDoFJoint, DiagnosticsWhenHangingHelper> helpers = new LinkedHashMap<OneDoFJoint, DiagnosticsWhenHangingHelper>();

   private final DoubleYoVariable ditherAmplitude = new DoubleYoVariable("ditherAmplitude", registry);
   private final DoubleYoVariable ditherFrequency = new DoubleYoVariable("ditherFrequency", registry);

   private final DoubleYoVariable maximumTorqueOffset = new DoubleYoVariable("maximumTorqueOffset", registry);

   private final BooleanYoVariable estimateTorqueOffset = new BooleanYoVariable("estimateTorqueOffset", registry);
   private final BooleanYoVariable transferTorqueOffsets = new BooleanYoVariable("transferTorqueOffsets", registry);
   private final BooleanYoVariable exportJointTorqueOffsetsToFile = new BooleanYoVariable("recordTorqueOffsets", registry);

   private final boolean useArms = true;

   private final TorqueOffsetPrinter torqueOffsetPrinter;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<>();

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.OFF);
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final BooleanYoVariable hasReachedMaximumTorqueOffset = new BooleanYoVariable("hasReachedMaximumTorqueOffset", registry);

   public JointTorqueOffsetEstimatorController(HighLevelHumanoidControllerToolbox highLevelControllerToolbox, TorqueOffsetPrinter torqueOffsetPrinter)
   {
      super(HighLevelState.CALIBRATION);

      this.bipedSupportPolygons = highLevelControllerToolbox.getBipedSupportPolygons();
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = highLevelControllerToolbox.getContactableFeet().get(robotSide);
         footContactStates.put(robotSide, highLevelControllerToolbox.getContactState(contactableFoot));
      }
      this.controllerToolbox = highLevelControllerToolbox;
      this.torqueOffsetPrinter = torqueOffsetPrinter;
      this.fullRobotModel = highLevelControllerToolbox.getFullRobotModel();

      ditherAmplitude.set(0.3);
      ditherFrequency.set(5.0);
      maximumTorqueOffset.set(5.0);

      estimateTorqueOffset.set(false);
      transferTorqueOffsets.set(false);

      fullRobotModel.getOneDoFJoints(oneDoFJoints);

      OneDoFJoint[] jointArray = fullRobotModel.getOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(jointArray);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(jointArray, LowLevelJointControlMode.FORCE_CONTROL);

      createHelpers(true);

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJoint joint = oneDoFJoints.get(i);

         if (!hasTorqueOffsetForJoint(joint))
            continue;

         String jointName = joint.getName();
         PDController controller = new PDController(jointName + "Calibration", registry);
         pdControllers.put(joint, controller);

         DoubleYoVariable desiredPosition = new DoubleYoVariable("q_d_calib_" + jointName, registry);
         desiredPositions.put(joint, desiredPosition);
      }

      initializeDesiredPositions();
      setDefaultPDControllerGains();
   }

   private final AtomicBoolean enableEstimationAtomic = new AtomicBoolean(false);
   private final AtomicBoolean disableEstimationAtomic = new AtomicBoolean(false);

   @Override
   public void enableJointTorqueOffsetEstimationAtomic(boolean enable)
   {
      if (enable)
         enableEstimationAtomic.set(true);
      else
         disableEstimationAtomic.set(true);
   }

   private void initializeDesiredPositions()
   {
      HumanoidJointPoseList humanoidJointPoseList = new HumanoidJointPoseList();
      humanoidJointPoseList.createCalibrationPose();
      SideDependentList<ArrayList<OneDoFJoint>> armJointList = humanoidJointPoseList.getArmJoints(fullRobotModel);
      SideDependentList<double[]> armJointAngleList = humanoidJointPoseList.getArmJointAngles();

      SideDependentList<ArrayList<OneDoFJoint>> legJointList = humanoidJointPoseList.getLegJoints(fullRobotModel);
      SideDependentList<double[]> legJointAngleList = humanoidJointPoseList.getLegJointAngles();

      ArrayList<OneDoFJoint> spineJoints = humanoidJointPoseList.getSpineJoints(fullRobotModel);
      double[] spineJointAngles = humanoidJointPoseList.getSpineJointAngles();

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<OneDoFJoint> armJoints = armJointList.get(robotSide);
         double[] armJointAngles = armJointAngleList.get(robotSide);

         for (int i = 0; i < armJoints.size(); i++)
         {
            OneDoFJoint armJoint = armJoints.get(i);
            if (hasTorqueOffsetForJoint(armJoint))
               desiredPositions.get(armJoint).set(armJointAngles[i]);
         }

         ArrayList<OneDoFJoint> legJoints = legJointList.get(robotSide);
         double[] legJointAngles = legJointAngleList.get(robotSide);

         for (int i = 0; i < legJoints.size(); i++)
         {
            OneDoFJoint legJoint = legJoints.get(i);
            if (hasTorqueOffsetForJoint(legJoint))
               desiredPositions.get(legJoint).set(legJointAngles[i]);
         }
      }

      for (int i = 0; i < spineJoints.size(); i++)
      {
         OneDoFJoint spineJoint = spineJoints.get(i);
         if (hasTorqueOffsetForJoint(spineJoint))
            desiredPositions.get(spineJoint).set(spineJointAngles[i]);
      }
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
      if (enableEstimationAtomic.getAndSet(false))
         estimateTorqueOffset.set(true);
      if (disableEstimationAtomic.getAndSet(false))
         estimateTorqueOffset.set(false);

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
         torqueOffsetPrinter.printTorqueOffsets(this);
      }

      lowLevelOneDoFJointDesiredDataHolder.setDesiredTorqueFromJoints(oneDoFJoints);
      controllerCoreCommand.completeLowLevelJointData(lowLevelOneDoFJointDesiredDataHolder);
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
         OneDoFJoint joint = oneDoFJoints.get(i);
         if (hasTorqueOffsetForJoint(joint))
         updatePDController(joint, getTimeInCurrentState());
      }
   }

   private void updatePDController(OneDoFJoint oneDoFJoint, double timeInCurrentState)
   {
      PDController pdController = pdControllers.get(oneDoFJoint);
      double desiredPosition = desiredPositions.get(oneDoFJoint).getDoubleValue();
      double desiredVelocity = 0.0;

      double tau = pdController.compute(oneDoFJoint.getQ(), desiredPosition, oneDoFJoint.getQd(), desiredVelocity);

      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
      if (diagnosticsWhenHangingHelper != null)
      {
         tau = diagnosticsWhenHangingHelper.getTorqueToApply(tau, estimateTorqueOffset.getBooleanValue(), maximumTorqueOffset.getDoubleValue());
         if (hasReachedMaximumTorqueOffset.getBooleanValue() && Math.abs(diagnosticsWhenHangingHelper.getTorqueOffset()) == maximumTorqueOffset.getDoubleValue())
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

      SideDependentList<InverseDynamicsJoint> topLegJoints = new SideDependentList<InverseDynamicsJoint>();
      for (RobotSide robotSide : RobotSide.values)
      {
         topLegJoints.set(robotSide, fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW));
      }

      OneDoFJoint spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW);
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
      OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
      helpers.put(armJoint, new DiagnosticsWhenHangingHelper(armJoint, preserveY, registry));
   }

   private void makeLegJointHelper(RobotSide robotSide, boolean preserveY, LegJointName legJointName)
   {
      OneDoFJoint legJoint = fullRobotModel.getLegJoint(robotSide, legJointName);
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

   @Override
   public void doAction()
   {
      doControl();
   }

   @Override
   public void doTransitionIntoAction()
   {
      initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {
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

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

   @Override
   public List<OneDoFJoint> getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   @Override
   public double getEstimatedJointTorqueOffset(OneDoFJoint joint)
   {
      DiagnosticsWhenHangingHelper helper = helpers.get(joint);
      return helper == null ? 0.0 : helper.getTorqueOffset();
   }

   @Override
   public void resetEstimatedJointTorqueOffset(OneDoFJoint joint)
   {
      if (hasTorqueOffsetForJoint(joint))
         helpers.get(joint).setTorqueOffset(0.0);
   }

   @Override
   public boolean hasTorqueOffsetForJoint(OneDoFJoint joint)
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
      return "Controller for estimating the joint torque offsets. It is based on " + DiagnosticsWhenHangingController.class.getSimpleName() + ".";
   }
}
