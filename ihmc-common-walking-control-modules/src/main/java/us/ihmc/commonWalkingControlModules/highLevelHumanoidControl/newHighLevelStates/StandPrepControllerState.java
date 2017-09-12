package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StandPrepControllerState extends NewHighLevelControllerState
{
   private static final NewHighLevelControllerStates controllerState = NewHighLevelControllerStates.STAND_PREP_STATE;
   private static final double MINIMUM_TIME_DONE_WITH_STAND_PREP = 0.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJoint, ImmutableTriple<YoDouble, YoPolynomial, YoDouble>> jointsData = new PairList<>();

   private final YoDouble timeToPrepareForStanding = new YoDouble("timeToPrepareForStanding", registry);
   private final YoDouble minimumTimeDoneWithStandPrep = new YoDouble("minimumTimeDoneWithStandPrep", registry);

   public StandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters)
   {
      this(controllerToolbox, highLevelControllerParameters, MINIMUM_TIME_DONE_WITH_STAND_PREP);
   }

   public StandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                   double minimumTimeDoneWithStandPrep)
   {
      super(controllerState);

      this.controllerToolbox = controllerToolbox;
      this.timeToPrepareForStanding.set(highLevelControllerParameters.getTimeToMoveInStandPrep());
      this.minimumTimeDoneWithStandPrep.set(minimumTimeDoneWithStandPrep);

      StandPrepParameters standPrepParameters = highLevelControllerParameters.getStandPrepParameters();
      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();

         YoPolynomial trajectory = new YoPolynomial(jointName + "_StandPrepTrajectory", 4, registry);
         YoDouble standPrepFinalConfiguration = new YoDouble(jointName + "_StandPrepFinalConfiguration", registry);
         YoDouble standPrepDesiredConfiguration = new YoDouble(jointName + "_StandPrepDesiredConfiguration", registry);

         standPrepFinalConfiguration.set(standPrepParameters.getSetpoint(jointName));

         ImmutableTriple<YoDouble, YoPolynomial, YoDouble> trajectorySet = new ImmutableTriple<>(standPrepFinalConfiguration, trajectory, standPrepDesiredConfiguration);
         jointsData.add(controlledJoint, trajectorySet);

         LowLevelJointControlMode jointControlMode = highLevelControllerParameters.getLowLevelJointControlMode(controlledJoint.getName(), controllerState);
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(controlledJoint, jointControlMode);
      }

   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJoint joint = jointsData.get(jointIndex).getLeft();
         ImmutableTriple<YoDouble, YoPolynomial, YoDouble> trajectoryData = jointsData.get(jointIndex).getRight();
         YoDouble standPrepFinal = trajectoryData.getLeft();
         YoPolynomial trajectory = trajectoryData.getMiddle();

         double desiredFinalPosition = standPrepFinal.getDoubleValue();
         double desiredFinalVelocity = 0.0;

         double currentAngle = joint.getQ();
         double currentVelocity = 0.0;

         trajectory.setCubic(0.0, timeToPrepareForStanding.getDoubleValue(), currentAngle, currentVelocity, desiredFinalPosition, desiredFinalVelocity);
      }
   }

   @Override
   public void doAction()
   {
      controllerToolbox.update();

      double timeInTrajectory = MathTools.clamp(getTimeInCurrentState(), 0.0, timeToPrepareForStanding.getDoubleValue());

      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJoint joint = jointsData.get(jointIndex).getLeft();
         ImmutableTriple<YoDouble, YoPolynomial, YoDouble> trajectoryData = jointsData.get(jointIndex).getRight();

         YoPolynomial trajectory = trajectoryData.getMiddle();
         YoDouble desiredPosition = trajectoryData.getRight();

         trajectory.compute(timeInTrajectory);
         desiredPosition.set(trajectory.getPosition());

         LowLevelJointData lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(joint);
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());
         lowLevelJointData.setDesiredVelocity(trajectory.getVelocity());
         lowLevelJointData.setDesiredAcceleration(trajectory.getAcceleration());
      }
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
   public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public void warmup(int iterations)
   {
   }
}
