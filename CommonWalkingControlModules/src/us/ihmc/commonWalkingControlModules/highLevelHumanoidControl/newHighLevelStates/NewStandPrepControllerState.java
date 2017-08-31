package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.ImmutableTriple;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator.EffortJointControlCalculator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator.PositionJointControlCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;

public class NewStandPrepControllerState extends NewHighLevelControllerState
{
   private static final double TIME_TO_SPLINE_TO_STAND_POSE = 4.0;
   private static final double MINIMUM_TIME_DONE_WITH_STAND_PREP = 1.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.OFF);

   private final OneDoFJoint[] controlledJoints;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJoint, ImmutablePair<ImmutablePair<YoDouble, YoPolynomial>, ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator>>> jointsData = new PairList<>();

   private final YoDouble timeToPrepareForStanding = new YoDouble("timeToPrepareForStanding", registry);
   private final YoDouble minimumTimeDoneWithStandPrep = new YoDouble("minimumTimeDoneWithStandPrep", registry);
   private final YoDouble masterGain = new YoDouble("standReadyMasterGain", registry);

   public NewStandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepParameters standPrepSetpoints,
                                      PositionControlParameters positionControlParameters)
   {
      this(controllerToolbox, standPrepSetpoints, positionControlParameters, TIME_TO_SPLINE_TO_STAND_POSE, MINIMUM_TIME_DONE_WITH_STAND_PREP);
   }

   public NewStandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepParameters standPrepSetpoints,
                                      PositionControlParameters positionControlParameters, double timeToPrepareForStanding, double minimumTimeDoneWithStandPrep)
   {
      super(NewHighLevelControllerStates.STAND_PREP_STATE);

      this.timeToPrepareForStanding.set(timeToPrepareForStanding);
      this.minimumTimeDoneWithStandPrep.set(minimumTimeDoneWithStandPrep);

      controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      masterGain.set(positionControlParameters.getPositionControlMasterGain());

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();

         YoPolynomial trajectory = new YoPolynomial(jointName + "_StandPrepTrajectory", 4, registry);
         YoDouble standPrepFinalConfiguration = new YoDouble(jointName + "_StandPrepFinalConfiguration", registry);

         EffortJointControlCalculator effortCalculator = new EffortJointControlCalculator("_StandReady", controlledJoint, controllerToolbox.getControlDT(), registry);
         PositionJointControlCalculator positionCalculator = new PositionJointControlCalculator("_StandReady", controlledJoint, registry);

         standPrepFinalConfiguration.set(standPrepSetpoints.getSetpoint(jointName));
         effortCalculator.setProportionalGain(positionControlParameters.getProportionalGain(jointName));
         effortCalculator.setDerivativeGain(positionControlParameters.getDerivativeGain(jointName));
         effortCalculator.setIntegralGain(positionControlParameters.getIntegralGain(jointName));

         ImmutablePair<YoDouble, YoPolynomial> trajectoryPair = new ImmutablePair<>(standPrepFinalConfiguration, trajectory);
         ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator> controllerPair = new ImmutablePair<>(effortCalculator, positionCalculator);

         ImmutablePair<ImmutablePair<YoDouble, YoPolynomial>, ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator>> jointData = new ImmutablePair<>(trajectoryPair, controllerPair);
         jointsData.add(controlledJoint, jointData);
      }

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJoint joint = jointsData.get(jointIndex).getLeft();
         ImmutablePair<YoDouble, YoPolynomial> trajectoryData = jointsData.get(jointIndex).getRight().getLeft();
         ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator> controllerData = jointsData.get(jointIndex).getRight().getRight();
         YoDouble standPrepSetpoint = trajectoryData.getLeft();
         YoPolynomial trajectory = trajectoryData.getRight();

         double desiredAngle = standPrepSetpoint.getDoubleValue();
         double desiredVelocity = 0.0;

         double currentAngle = joint.getQ();
         double currentVelocity = joint.getQd();

         trajectory.setCubic(0.0, timeToPrepareForStanding.getDoubleValue(), currentAngle, currentVelocity, desiredAngle, desiredVelocity);

         EffortJointControlCalculator effortController = controllerData.getLeft();
         PositionJointControlCalculator positionController = controllerData.getRight();
         effortController.initialize();
         positionController.initialize();
      }
   }

   @Override
   public void doAction()
   {
      double timeInTrajectory = MathTools.clamp(getTimeInCurrentState(), 0.0, timeToPrepareForStanding.getDoubleValue());

      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJoint joint = jointsData.get(jointIndex).getLeft();
         ImmutablePair<YoDouble, YoPolynomial> trajectoryData = jointsData.get(jointIndex).getRight().getLeft();
         ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator> controllerData = jointsData.get(jointIndex).getRight().getRight();

         YoPolynomial trajectory = trajectoryData.getRight();
         trajectory.compute(timeInTrajectory);

         LowLevelJointData lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(joint);
         lowLevelJointData.setDesiredPosition(trajectory.getPosition());
         lowLevelJointData.setDesiredVelocity(trajectory.getVelocity());

         EffortJointControlCalculator effortCalculator = controllerData.getLeft();
         PositionJointControlCalculator positionCalculator = controllerData.getRight();

         effortCalculator.computeAndUpdateJointTorque(lowLevelJointData, masterGain.getDoubleValue());
         positionCalculator.computeAndUpdateJointPosition(lowLevelJointData, masterGain.getDoubleValue());
      }

      controllerCoreCommand.completeLowLevelJointData(lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // Do nothing

   }

   @Override
   public boolean isDone()
   {
      return getTimeInCurrentState() > (timeToPrepareForStanding.getDoubleValue() + minimumTimeDoneWithStandPrep.getDoubleValue());
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

}
