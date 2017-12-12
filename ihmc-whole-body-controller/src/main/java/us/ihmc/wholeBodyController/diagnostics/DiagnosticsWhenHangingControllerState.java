package us.ihmc.wholeBodyController.diagnostics;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.wholeBodyController.JointTorqueOffsetProcessor;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class DiagnosticsWhenHangingControllerState extends HighLevelControllerState implements RobotController, JointTorqueOffsetEstimator
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.DIAGNOSTICS;

   private JointTorqueOffsetProcessor jointTorqueOffsetProcessor;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   private FullHumanoidRobotModel fullRobotModel;
   private final ArrayList<OneDoFJoint> oneDoFJoints = new ArrayList<OneDoFJoint>();

   private final LinkedHashMap<OneDoFJoint, PDController> pdControllers = new LinkedHashMap<OneDoFJoint, PDController>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> initialPositions = new LinkedHashMap<OneDoFJoint, YoDouble>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> finalPositions = new LinkedHashMap<OneDoFJoint, YoDouble>();
   private final LinkedHashMap<OneDoFJoint, YoMinimumJerkTrajectory> transitionSplines = new LinkedHashMap<OneDoFJoint, YoMinimumJerkTrajectory>();

   private final YoBoolean manualMode = new YoBoolean("diagnosticsWhenHangingManualMode", registry);
   private final LinkedHashMap<OneDoFJoint, YoDouble> desiredPositions = new LinkedHashMap<OneDoFJoint, YoDouble>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> desiredVelocities = new LinkedHashMap<OneDoFJoint, YoDouble>();

   private final LinkedHashMap<OneDoFJoint, DiagnosticsWhenHangingHelper> helpers = new LinkedHashMap<OneDoFJoint, DiagnosticsWhenHangingHelper>();
   private final LinkedHashMap<OneDoFJoint, Double> torqueOffsetSigns = new LinkedHashMap<OneDoFJoint, Double>();

   private YoDouble yoTime;

   private final YoBoolean startDiagnostics = new YoBoolean("startDiagnostics", registry);
   private final YoBoolean pauseDiagnostics = new YoBoolean("pauseDiagnostics", registry);
   private final YoBoolean finishedDiagnostics = new YoBoolean("finishedDiagnostics", registry);

   private final YoDouble splineDuration = new YoDouble("splineDuration", registry);

   private final YoDouble ditherAmplitude = new YoDouble("ditherAmplitude", registry);
   private final YoDouble ditherFrequency = new YoDouble("ditherFrequency", registry);

   private final YoDouble diagnosticsPDMasterGain = new YoDouble("diagnosticsPDMasterGain", registry);
   private final YoDouble maximumTorqueOffset = new YoDouble("maximumTorqueOffset", registry);

   private final YoBoolean adaptTorqueOffset = new YoBoolean("adaptTorqueOffset", registry);
   private final YoBoolean printTorqueOffsets = new YoBoolean("printTorqueOffsets", registry);
   private final YoBoolean transferTorqueOffsets = new YoBoolean("transferTorqueOffsets", registry);

   private StateMachine<DiagnosticsWhenHangingState> stateMachine;

   private final boolean useArms;

   private final YoBoolean updateFootForceSensorOffsets = new YoBoolean("updateFootForceSensorOffsets", registry);
   private final YoBoolean printForceSensorsOffsets = new YoBoolean("printForceSensorsOffsets", registry);
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<YoFrameVector> footForcesRaw = new SideDependentList<YoFrameVector>();
   private final SideDependentList<YoFrameVector> footTorquesRaw = new SideDependentList<YoFrameVector>();
   private final YoDouble alphaFootForce = new YoDouble("alphaDiagFootForce", registry);
   private final SideDependentList<AlphaFilteredYoFrameVector> footForcesRawFiltered = new SideDependentList<AlphaFilteredYoFrameVector>();
   private final SideDependentList<AlphaFilteredYoFrameVector> footTorquesRawFiltered = new SideDependentList<AlphaFilteredYoFrameVector>();

   private final TorqueOffsetPrinter torqueOffsetPrinter;

   private final HumanoidJointPoseList humanoidJointPoseList;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<YoPlaneContactState> footContactStates;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   public DiagnosticsWhenHangingControllerState(HumanoidJointPoseList humanoidJointPoseList, boolean useArms, boolean robotIsHanging,
                                                HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                                TorqueOffsetPrinter torqueOffsetPrinter)
   {
      super(controllerState, highLevelControllerParameters, controllerToolbox);

      this.humanoidJointPoseList = humanoidJointPoseList;
      this.bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();
      this.footContactStates = controllerToolbox.getFootContactStates();
      this.controllerToolbox = controllerToolbox;
      humanoidJointPoseList.setParentRegistry(registry);

      splineDuration.set(3.0);

      this.useArms = useArms;

      this.torqueOffsetPrinter = torqueOffsetPrinter;

      ditherAmplitude.set(0.3);
      ditherFrequency.set(5.0);

      diagnosticsPDMasterGain.set(1.0);

      maximumTorqueOffset.set(15.0);

      adaptTorqueOffset.set(false);
      transferTorqueOffsets.set(false);

      this.yoTime = controllerToolbox.getYoTime();
      this.fullRobotModel = controllerToolbox.getFullRobotModel();
      fullRobotModel.getOneDoFJoints(oneDoFJoints);

      OneDoFJoint[] jointArray = ScrewTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJoint.class);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(jointArray);

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints.get(i);
         YoDouble initialPosition = new YoDouble(oneDoFJoint.getName() + "InitialPosition", registry);
         YoDouble finalPosition = new YoDouble(oneDoFJoint.getName() + "FinalPosition", registry);
         initialPosition.set(oneDoFJoint.getQ());
         finalPosition.set(oneDoFJoint.getQ());

         initialPositions.put(oneDoFJoint, initialPosition);
         finalPositions.put(oneDoFJoint, finalPosition);

         //       System.out.println("Setting initial and final positions for " + oneDoFJoint.getName() + " to " + oneDoFJoint.getQ());
      }

      setDesiredPositionsFromPoseList(finalPositions);

      copyFinalDesiredPositionsToInitialDesired();

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         PDController controller = new PDController(oneDoFJoints.get(i).getName(), registry);
         pdControllers.put(oneDoFJoints.get(i), controller);
      }

      setDefaultPDControllerGains();
      createTransitionSplines();

      createHelpers(robotIsHanging);
      setTransitionSplines();

      stateMachine = new StateMachine<DiagnosticsWhenHangingState>("DiagnosticsState", "DiagnosticsSwitchTime", DiagnosticsWhenHangingState.class, yoTime,
            registry);

      InitializeState initializeState = new InitializeState(DiagnosticsWhenHangingState.INITIALIZE);
      initializeState.setDefaultNextState(DiagnosticsWhenHangingState.SPLINE_BETWEEN_POSITIONS);
      stateMachine.addState(initializeState);

      SplineBetweenPositionsState splineBetweenPositionsState = new SplineBetweenPositionsState(DiagnosticsWhenHangingState.SPLINE_BETWEEN_POSITIONS);
      splineBetweenPositionsState.setDefaultNextState(DiagnosticsWhenHangingState.CHECK_DIAGNOSTICS);
      stateMachine.addState(splineBetweenPositionsState);

      CheckDiagnosticsState checkDiagnosticsState = new CheckDiagnosticsState(DiagnosticsWhenHangingState.CHECK_DIAGNOSTICS);
      checkDiagnosticsState.setDefaultNextState(DiagnosticsWhenHangingState.SPLINE_BETWEEN_POSITIONS);

      StateTransitionCondition finishedTransitionCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return finishedDiagnostics.getBooleanValue();
         }
      };

      StateTransition<DiagnosticsWhenHangingState> transitionToFinished = new StateTransition<DiagnosticsWhenHangingState>(DiagnosticsWhenHangingState.FINISHED,
                                                                                                                           finishedTransitionCondition);
      checkDiagnosticsState.addStateTransition(transitionToFinished);

      stateMachine.addState(checkDiagnosticsState);

      FinishedState finishedState = new FinishedState(DiagnosticsWhenHangingState.FINISHED);
      finishedState.setDefaultNextState(DiagnosticsWhenHangingState.INITIALIZE);
      stateMachine.addState(finishedState);

      // Foot force sensors tarring stuff
      footSwitches = controllerToolbox.getFootSwitches();
      alphaFootForce.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.1, controllerToolbox.getControlDT()));
      updateFootForceSensorOffsets.set(true);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         ReferenceFrame footSensorFrame = footSwitches.get(robotSide).getMeasurementFrame();

         YoFrameVector footForceRaw = new YoFrameVector(sidePrefix + "DiagFootForceRaw", footSensorFrame, registry);
         footForcesRaw.put(robotSide, footForceRaw);

         YoFrameVector footTorqueRaw = new YoFrameVector(sidePrefix + "DiagFootTorqueRaw", footSensorFrame, registry);
         footTorquesRaw.put(robotSide, footTorqueRaw);

         AlphaFilteredYoFrameVector footForceRawFiltered = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(sidePrefix + "DiagFootForceRawFilt", "",
               registry, alphaFootForce, footForceRaw);
         footForcesRawFiltered.put(robotSide, footForceRawFiltered);

         AlphaFilteredYoFrameVector footTorqueRawFiltered = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(sidePrefix + "DiagFootTorqueRawFilt",
               "", registry, alphaFootForce, footTorqueRaw);
         footTorquesRawFiltered.put(robotSide, footTorqueRawFiltered);
      }
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   @Override
   public ArrayList<OneDoFJoint> getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   public DiagnosticsWhenHangingHelper getDiagnosticsWhenHangingHelper(OneDoFJoint oneDoFJoint)
   {
      return helpers.get(oneDoFJoint);
   }

   public double getTorqueOffsetSign(OneDoFJoint oneDoFJoint)
   {
      return torqueOffsetSigns.get(oneDoFJoint);
   }

   @Override
   public String getDescription()
   {
      return "Controller for doing diagnostics when robot is hanging in the air. It will go to several known configurations and check that the joint torques are reasonable.";
   }

   @Override
   public void doControl()
   {
      callUpdatables();

      stateMachine.doAction();
      updateDiagnosticsWhenHangingHelpers();
      updatePDControllers();
      if (updateFootForceSensorOffsets.getBooleanValue())
         updateFootSensorRawMeasurement();
      stateMachine.checkTransitionConditions();

      if (transferTorqueOffsets.getBooleanValue())
      {
         transferTorqueOffsets.set(false);
         transferTorqueOffsetsToOutputWriter();
      }

      if (printTorqueOffsets.getBooleanValue() && torqueOffsetPrinter != null)
      {
         printTorqueOffsets.set(false);
         //         System.err.println("Need to reimplement this again!");
         torqueOffsetPrinter.printTorqueOffsets(this);
         //         printOffsetsForCoeffsForValkyrie();
      }

      if (printForceSensorsOffsets.getBooleanValue())
      {
         printForceSensorsOffsets.set(false);
         printFootSensorsOffset();
      }

      OneDoFJoint[] jointArray = fullRobotModel.getOneDoFJoints();
      for (int jointIdx = 0; jointIdx < jointArray.length; jointIdx++)
      {
         OneDoFJoint joint = jointArray[jointIdx];
         JointDesiredOutput jointDesiredOutput = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         jointDesiredOutput.clear();
         jointDesiredOutput.setDesiredTorque(joint.getTau());
      }
      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   private void callUpdatables()
   {
      double time = yoTime.getDoubleValue();
      for (int i = 0; i < updatables.size(); i++)
      {
         updatables.get(i).update(time);
      }

      bipedSupportPolygons.updateUsingContactStates(footContactStates);
      controllerToolbox.update();
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

   public void addOffsetTorquesToAppliedTorques()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoints.get(i));
         if (diagnosticsWhenHangingHelper != null)
            diagnosticsWhenHangingHelper.addOffsetToEstimatedTorque();
      }
   }

   public double getEstimatedTorque(OneDoFJoint oneDoFJoint)
   {
      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
      if (diagnosticsWhenHangingHelper != null)
         return diagnosticsWhenHangingHelper.getEstimatedTorque();

      return 0.0;
   }

   public double getAppliedTorque(OneDoFJoint oneDoFJoint)
   {
      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
      if (diagnosticsWhenHangingHelper != null)
         return diagnosticsWhenHangingHelper.getAppliedTorque();

      return 0.0;
   }

   public YoDouble getEstimatedTorqueYoVariable(OneDoFJoint oneDoFJoint)
   {
      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
      if (diagnosticsWhenHangingHelper != null)
         return diagnosticsWhenHangingHelper.getEstimatedTorqueYoVariable();

      return null;
   }

   public YoDouble getAppliedTorqueYoVariable(OneDoFJoint oneDoFJoint)
   {
      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
      if (diagnosticsWhenHangingHelper != null)
         return diagnosticsWhenHangingHelper.getAppliedTorqueYoVariable();

      return null;
   }

   private void updatePDControllers()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         updatePDController(oneDoFJoints.get(i), stateMachine.timeInCurrentState());
      }
   }

   private void setDesiredPositionsFromPoseList(LinkedHashMap<OneDoFJoint, YoDouble> positionListToSet)
   {
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
            positionListToSet.get(armJoint).set(armJointAngles[i]);
         }

         ArrayList<OneDoFJoint> legJoints = legJointList.get(robotSide);
         double[] legJointAngles = legJointAngleList.get(robotSide);

         for (int i = 0; i < legJoints.size(); i++)
         {
            OneDoFJoint legJoint = legJoints.get(i);
            positionListToSet.get(legJoint).set(legJointAngles[i]);
         }

      }

      for (int i = 0; i < spineJoints.size(); i++)
      {
         OneDoFJoint spineJoint = spineJoints.get(i);
         positionListToSet.get(spineJoint).set(spineJointAngles[i]);
      }
   }

   private class InitializeState extends State<DiagnosticsWhenHangingState>
   {
      public InitializeState(DiagnosticsWhenHangingState stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         if (startDiagnostics.getBooleanValue())
            this.transitionToDefaultNextState();

         for (int i = 0; i < oneDoFJoints.size(); i++)
         {
            updateTrajectory(oneDoFJoints.get(i), stateMachine.timeInCurrentState());
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         humanoidJointPoseList.reset();
         finishedDiagnostics.set(false);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         //       startDiagnostics.set(false);
      }
   }

   private class SplineBetweenPositionsState extends State<DiagnosticsWhenHangingState>
   {
      public SplineBetweenPositionsState(DiagnosticsWhenHangingState stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         for (int i = 0; i < oneDoFJoints.size(); i++)
         {
            updateTrajectory(oneDoFJoints.get(i), stateMachine.timeInCurrentState());
         }

         if (pauseDiagnostics.getBooleanValue())
            return;
         if (splinesAreFinished(stateMachine.timeInCurrentState()))
            this.transitionToDefaultNextState();
      }

      private boolean splinesAreFinished(double timeInCurrentState)
      {
         for (int i = 0; i < oneDoFJoints.size(); i++)
         {
            YoMinimumJerkTrajectory spline = transitionSplines.get(oneDoFJoints.get(i));
            if (timeInCurrentState < spline.getFinalTime())
               return false;
         }

         return true;
      }

      @Override
      public void doTransitionIntoAction()
      {
         copyFinalDesiredPositionsToInitialDesired();
         setDesiredPositionsFromPoseList(finalPositions);
         setTransitionSplines();
      }

      @Override
      public void doTransitionOutOfAction()
      {
         humanoidJointPoseList.next();
      }

   }

   private class CheckDiagnosticsState extends State<DiagnosticsWhenHangingState>
   {
      public CheckDiagnosticsState(DiagnosticsWhenHangingState stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         if (stateMachine.timeInCurrentState() > 1.0)
         {
            if (!humanoidJointPoseList.isDone())
               this.transitionToDefaultNextState();
            else
               finishedDiagnostics.set(true);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }

   }

   private class FinishedState extends State<DiagnosticsWhenHangingState>
   {
      public FinishedState(DiagnosticsWhenHangingState stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         if ((stateMachine.timeInCurrentState() > 5.0) && startDiagnostics.getBooleanValue())
            this.transitionToDefaultNextState();
      }

      @Override
      public void doTransitionIntoAction()
      {
         zeroAllTorques();
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }

   }

   private void updatePDController(OneDoFJoint oneDoFJoint, double timeInCurrentState)
   {
      PDController pdController = pdControllers.get(oneDoFJoint);
      YoMinimumJerkTrajectory transitionSpline = transitionSplines.get(oneDoFJoint);
      double desiredPosition = transitionSpline.getPosition();
      double desiredVelocity = transitionSpline.getVelocity();

      // Setting the desired positions via SCS ui.
      if (manualMode.getBooleanValue())
      {
         desiredPosition = desiredPositions.get(oneDoFJoint).getDoubleValue();
         desiredVelocity = 0.0;
      }
      else
      {
         desiredPositions.get(oneDoFJoint).set(desiredPosition);
      }

      desiredVelocities.get(oneDoFJoint).set(desiredVelocity);

      double tau = pdController.compute(oneDoFJoint.getQ(), desiredPosition, oneDoFJoint.getQd(), desiredVelocity);
      tau = tau * diagnosticsPDMasterGain.getDoubleValue();

      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
      if (diagnosticsWhenHangingHelper != null)
      {
         tau = diagnosticsWhenHangingHelper.getTorqueToApply(tau, adaptTorqueOffset.getBooleanValue(), maximumTorqueOffset.getDoubleValue());
      }

      double ditherTorque = ditherAmplitude.getDoubleValue() * Math.sin(2.0 * Math.PI * ditherFrequency.getDoubleValue() * timeInCurrentState);
      oneDoFJoint.setTau(tau + ditherTorque);
   }

   private void updateTrajectory(OneDoFJoint oneDoFJoint, double timeInCurrentState)
   {
      YoMinimumJerkTrajectory transitionSpline = transitionSplines.get(oneDoFJoint);
      transitionSpline.computeTrajectory(timeInCurrentState);
   }

   private void createTransitionSplines()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints.get(i);
         YoMinimumJerkTrajectory spline = new YoMinimumJerkTrajectory(oneDoFJoint.getName() + "TransitionSpline", registry);
         transitionSplines.put(oneDoFJoint, spline);

         //       spline.setParams(initialPositions.get(oneDoFJoint), 0.0, 0.0, finalPositions.get(oneDoFJoint), 0.0, 0.0, 0.0, 3.0);

         YoDouble desiredPosition = new YoDouble("q_d_" + oneDoFJoint.getName(), registry);
         YoDouble desiredVelocity = new YoDouble("qd_d_" + oneDoFJoint.getName(), registry);

         desiredPositions.put(oneDoFJoint, desiredPosition);
         desiredVelocities.put(oneDoFJoint, desiredVelocity);
      }
   }

   private void setTransitionSplines()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints.get(i);
         YoMinimumJerkTrajectory spline = transitionSplines.get(oneDoFJoint);
         double initialPosition = initialPositions.get(oneDoFJoint).getDoubleValue();
         double finalPositon = finalPositions.get(oneDoFJoint).getDoubleValue();
         spline.setParams(initialPosition, 0.0, 0.0, finalPositon, 0.0, 0.0, 0.0, splineDuration.getDoubleValue());
      }
   }

   private void copyFinalDesiredPositionsToInitialDesired()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         YoDouble initialPosition = initialPositions.get(oneDoFJoints.get(i));
         YoDouble finalPosition = finalPositions.get(oneDoFJoints.get(i));
         initialPosition.set(finalPosition.getDoubleValue());
      }
   }

   private void zeroAllTorques()
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         oneDoFJoints.get(i).setTau(0);
      }
   }

   private void createHelpers(boolean robotIsHanging)
   {
      if (useArms)
      {
         makeArmJointHelper(RobotSide.LEFT, 1.0, true, ArmJointName.SHOULDER_PITCH);
         makeArmJointHelper(RobotSide.RIGHT, -1.0, true, ArmJointName.SHOULDER_PITCH);

         makeArmJointHelper(RobotSide.LEFT, -1.0, false, ArmJointName.SHOULDER_ROLL);
         makeArmJointHelper(RobotSide.RIGHT, 1.0, false, ArmJointName.SHOULDER_ROLL);

         makeArmJointHelper(RobotSide.LEFT, 1.0, false, ArmJointName.SHOULDER_YAW);
         makeArmJointHelper(RobotSide.RIGHT, -1.0, false, ArmJointName.SHOULDER_YAW);

         makeArmJointHelper(RobotSide.LEFT, -1.0, true, ArmJointName.ELBOW_PITCH); // TODO: Should this be false here?
         makeArmJointHelper(RobotSide.RIGHT, -1.0, true, ArmJointName.ELBOW_PITCH);
      }

      SideDependentList<InverseDynamicsJoint> topLegJoints = new SideDependentList<InverseDynamicsJoint>();
      for (RobotSide robotSide : RobotSide.values)
      {
         topLegJoints.set(robotSide, fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW));
      }

      OneDoFJoint spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW);
      helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, false, robotIsHanging, topLegJoints, registry));
      torqueOffsetSigns.put(spineJoint, 1.0);

      spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);
      helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, true, robotIsHanging, topLegJoints, registry));
      torqueOffsetSigns.put(spineJoint, 1.0);

      spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, false, robotIsHanging, topLegJoints, registry));
      torqueOffsetSigns.put(spineJoint, 1.0);

      makeLegJointHelper(RobotSide.LEFT, 1.0, false, LegJointName.HIP_YAW);
      makeLegJointHelper(RobotSide.RIGHT, -1.0, false, LegJointName.HIP_YAW);

      makeLegJointHelper(RobotSide.LEFT, 1.0, true, LegJointName.HIP_PITCH);
      makeLegJointHelper(RobotSide.RIGHT, -1.0, true, LegJointName.HIP_PITCH);

      makeLegJointHelper(RobotSide.LEFT, 1.0, false, LegJointName.HIP_ROLL);
      makeLegJointHelper(RobotSide.RIGHT, -1.0, false, LegJointName.HIP_ROLL);

      makeLegJointHelper(RobotSide.LEFT, 1.0, true, LegJointName.KNEE_PITCH);
      makeLegJointHelper(RobotSide.RIGHT, -1.0, true, LegJointName.KNEE_PITCH);

      makeLegJointHelper(RobotSide.LEFT, 1.0, true, LegJointName.ANKLE_PITCH);
      makeLegJointHelper(RobotSide.RIGHT, 1.0, true, LegJointName.ANKLE_PITCH);

      makeLegJointHelper(RobotSide.LEFT, 1.0, false, LegJointName.ANKLE_ROLL);
      makeLegJointHelper(RobotSide.RIGHT, 1.0, false, LegJointName.ANKLE_ROLL);
   }

   private void makeArmJointHelper(RobotSide robotSide, double torqueOffsetSign, boolean preserveY, ArmJointName armJointName)
   {
      OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
      helpers.put(armJoint, new DiagnosticsWhenHangingHelper(armJoint, preserveY, registry));
      torqueOffsetSigns.put(armJoint, torqueOffsetSign);
   }

   private void makeLegJointHelper(RobotSide robotSide, double torqueOffsetSign, boolean preserveY, LegJointName legJointName)
   {
      OneDoFJoint legJoint = fullRobotModel.getLegJoint(robotSide, legJointName);
      helpers.put(legJoint, new DiagnosticsWhenHangingHelper(legJoint, preserveY, registry));
      torqueOffsetSigns.put(legJoint, torqueOffsetSign);
   }

   public double[] getAnkleTorqueOffsets(RobotSide robotSide)
   {
      OneDoFJoint anklePitch = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
      OneDoFJoint ankleRoll = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL);

      double anklePitchTorqueOffset = helpers.get(anklePitch).getTorqueOffset();
      double anklePitchTorqueOffsetSign = torqueOffsetSigns.get(anklePitch);

      double ankleRollTorqueOffset = helpers.get(ankleRoll).getTorqueOffset();
      double ankleRollTorqueOffsetSign = torqueOffsetSigns.get(ankleRoll);

      return new double[] { anklePitchTorqueOffset * anklePitchTorqueOffsetSign, ankleRollTorqueOffset * ankleRollTorqueOffsetSign };
   }

   public double[] getWaistTorqueOffsets()
   {
      OneDoFJoint waistPitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);
      OneDoFJoint waistRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);

      double waistPitchTorqueOffset = helpers.get(waistPitch).getTorqueOffset();
      double waistPitchTorqueOffsetSign = torqueOffsetSigns.get(waistPitch);

      double waistRollTorqueOffset = helpers.get(waistRoll).getTorqueOffset();
      double waistRollTorqueOffsetSign = torqueOffsetSigns.get(waistRoll);

      return new double[] { waistPitchTorqueOffset * waistPitchTorqueOffsetSign, waistRollTorqueOffset * waistRollTorqueOffsetSign };
   }

   public ArrayList<YoDouble> getTorqueOffsetVariables()
   {
      ArrayList<YoDouble> torqueOffsetVariables = new ArrayList<YoDouble>();

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoints.get(i));
         if (diagnosticsWhenHangingHelper != null)
         {
            YoDouble torqueOffsetVariable = diagnosticsWhenHangingHelper.getTorqueOffsetVariable();
            torqueOffsetVariables.add(torqueOffsetVariable);
         }
      }

      return torqueOffsetVariables;
   }

   public YoDouble getTorqueOffsetVariable(OneDoFJoint oneDoFJoint)
   {
      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
      if (diagnosticsWhenHangingHelper != null)
      {
         return diagnosticsWhenHangingHelper.getTorqueOffsetVariable();
      }

      return null;
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

   public enum DiagnosticsWhenHangingState
   {
      INITIALIZE, SPLINE_BETWEEN_POSITIONS, CHECK_DIAGNOSTICS, FINISHED
   }

   public void addUpdatable(Updatable updatable)
   {
      this.updatables.add(updatable);
   }

   public void addUpdatables(ArrayList<Updatable> updatables)
   {
      this.updatables.addAll(updatables);
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

   public void attachjointTorqueOffsetProcessor(JointTorqueOffsetProcessor jointTorqueOffsetProcessor)
   {
      this.jointTorqueOffsetProcessor = jointTorqueOffsetProcessor;
   }

   public void transferTorqueOffsetsToOutputWriter()
   {
      if (jointTorqueOffsetProcessor == null)
         return;

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoints.get(i));

         if (diagnosticsWhenHangingHelper != null)
         {
            double torqueOffset = diagnosticsWhenHangingHelper.getTorqueOffset();
            jointTorqueOffsetProcessor.subtractTorqueOffset(oneDoFJoints.get(i), torqueOffset);

            diagnosticsWhenHangingHelper.setTorqueOffset(0.0);
         }
      }
   }

   private final Wrench tempWrench = new Wrench();
   private final FrameVector3D tempFrameVector = new FrameVector3D();

   private void updateFootSensorRawMeasurement()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FootSwitchInterface footSwitch = footSwitches.get(robotSide);
         tempWrench.setToZero(footSwitch.getMeasurementFrame(), footSwitch.getMeasurementFrame());
         tempFrameVector.setToZero(footSwitch.getMeasurementFrame());

         footSwitch.computeAndPackFootWrench(tempWrench);
         tempWrench.getLinearPart(tempFrameVector);
         footForcesRaw.get(robotSide).set(tempFrameVector);
         tempWrench.getAngularPart(tempFrameVector);
         footTorquesRaw.get(robotSide).set(tempFrameVector);

         footForcesRawFiltered.get(robotSide).update();
         footTorquesRawFiltered.get(robotSide).update();
      }
   }

   private void printFootSensorsOffset()
   {
      java.text.NumberFormat doubleFormat = new java.text.DecimalFormat(" 0.00;-0.00");

      String offsetString = "";

      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());

      offsetString += "Copy the following in ValkyrieSensorInformation:\n";

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForStartOfExpression();
         offsetString += "      SpatialForceVector " + side + "FootForceSensorTareOffset_" + timestamp + " = new SpatialForceVector(null, new double[] {";
         offsetString += doubleFormat.format(footTorquesRawFiltered.get(robotSide).getX()) + ", ";
         offsetString += doubleFormat.format(footTorquesRawFiltered.get(robotSide).getY()) + ", ";
         offsetString += doubleFormat.format(footTorquesRawFiltered.get(robotSide).getZ()) + ", ";
         offsetString += doubleFormat.format(footForcesRawFiltered.get(robotSide).getX()) + ", ";
         offsetString += doubleFormat.format(footForcesRawFiltered.get(robotSide).getY()) + ", ";
         offsetString += doubleFormat.format(footForcesRawFiltered.get(robotSide).getZ()) + "});\n";
      }

      offsetString += "\n      footForceSensorTareOffsets = new SideDependentList<SpatialForceVector>(leftFootForceSensorTareOffset_" + timestamp
            + ", rightFootForceSensorTareOffset_" + timestamp + ");";

      System.out.println(offsetString);
   }

   public void setAppliedTorque(OneDoFJoint oneDoFJoint, double appliedTorque)
   {
      DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);

      if (diagnosticsWhenHangingHelper != null)
      {
         diagnosticsWhenHangingHelper.setAppliedTorque(appliedTorque);
      }
   }

   // private void tareFootSensors()
   // {
   //    for (RobotSide robotSide : RobotSide.values)
   //    {
   //       FootSwitchInterface footSwitch = footSwitches.get(robotSide);
   //       tempWrench.setToZero(footSwitch.getMeasurementFrame(), footSwitch.getMeasurementFrame());
   //
   //       footForcesRawFiltered.get(robotSide).getFrameTupleIncludingFrame(tempFrameVector);
   //       tempWrench.setLinearPart(tempFrameVector);
   //       footTorquesRawFiltered.get(robotSide).getFrameTupleIncludingFrame(tempFrameVector);
   //       tempWrench.setAngularPart(tempFrameVector);
   //
   //       footSwitch.setSensorWrenchOffset(tempWrench);
   //    }
   // }

   @Override
   public double getEstimatedJointTorqueOffset(OneDoFJoint joint)
   {
      DiagnosticsWhenHangingHelper helper = helpers.get(joint);
      return helper == null ? Double.NaN : helper.getTorqueOffset();
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
}
