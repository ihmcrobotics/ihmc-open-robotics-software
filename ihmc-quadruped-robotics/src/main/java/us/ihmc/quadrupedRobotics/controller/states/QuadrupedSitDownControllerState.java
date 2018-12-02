package us.ihmc.quadrupedRobotics.controller.states;

import com.google.common.base.CaseFormat;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedSitDownParameters;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSitDownControllerState extends HighLevelControllerState
{
   public static final String name = "SIT_DOWN";

   private static final double MINIMUM_TIME_DONE_WITH_SITTING_DOWN = 0.0;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJointBasics, TrajectoryData> jointsData = new PairList<>();

   private final YoDouble timeToMoveSittingDown = new YoDouble("timeToMoveSittingDown", registry);
   private final YoDouble minimumTimeDoneWithSittingDown = new YoDouble("minimumTimeDoneWithSittingDown", registry);
   private final JointDesiredOutputListReadOnly highLevelControlOutput;

   public QuadrupedSitDownControllerState(HighLevelControllerName controllerState, OneDoFJointBasics[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters,
                                          QuadrupedSitDownParameters sitDownParameters, JointDesiredOutputListReadOnly highLevelControlOutput)
   {
      this(controllerState, controlledJoints, highLevelControllerParameters, sitDownParameters, highLevelControlOutput, MINIMUM_TIME_DONE_WITH_SITTING_DOWN);
   }

   public QuadrupedSitDownControllerState(HighLevelControllerName controllerState, OneDoFJointBasics[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters,
                                          QuadrupedSitDownParameters sitDownParameters, JointDesiredOutputListReadOnly highLevelControlOutput,
                                          double minimumTimeDoneWithSittingDown)
   {
      super(controllerState, highLevelControllerParameters, controlledJoints);
      this.highLevelControlOutput = highLevelControlOutput;

      this.timeToMoveSittingDown.set(sitDownParameters.getTimeToMoveForSittingDown());
      this.minimumTimeDoneWithSittingDown.set(minimumTimeDoneWithSittingDown);

      WholeBodySetpointParameters sitDownConfiguration = sitDownParameters.getSitDownParameters();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();
         String namePrefix = CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, jointName);

         YoPolynomial trajectory = new YoPolynomial(namePrefix + "SitDownTrajectory", 4, registry);
         DoubleProvider sitDownFinalConfiguration = new DoubleParameter(namePrefix + "SitDownPosition", registry,
                                                                          sitDownConfiguration.getSetpoint(jointName));
         YoDouble sitDownDesiredConfiguration = new YoDouble(namePrefix + "SittingDownCurrentDesired", registry);

         TrajectoryData jointData = new TrajectoryData(sitDownFinalConfiguration, sitDownDesiredConfiguration, trajectory);
         jointsData.add(controlledJoint, jointData);
      }
   }

   @Override
   public void onEntry()
   {
      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointsData.get(jointIndex).getLeft();
         TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();
         DoubleProvider sitDownFinal = trajectoryData.getFinalJointConfiguration();
         YoPolynomial trajectory = trajectoryData.getJointTrajectory();

         double desiredFinalPosition = sitDownFinal.getValue();
         double desiredFinalVelocity = 0.0;

         JointDesiredOutputReadOnly jointDesiredOutput = highLevelControlOutput.getJointDesiredOutput(joint);
         double startAngle;
         if (jointDesiredOutput != null && jointDesiredOutput.hasDesiredPosition())
            startAngle = jointDesiredOutput.getDesiredPosition();
         else
            startAngle = joint.getQ();
         double startVelocity = 0.0;

         trajectory.setCubic(0.0, timeToMoveSittingDown.getDoubleValue(), startAngle, startVelocity, desiredFinalPosition, desiredFinalVelocity);
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = MathTools.clamp(timeInState, 0.0, timeToMoveSittingDown.getDoubleValue());

      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointsData.get(jointIndex).getLeft();
         TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

         YoPolynomial trajectory = trajectoryData.getJointTrajectory();
         YoDouble desiredPosition = trajectoryData.getDesiredJointConfiguration();

         trajectory.compute(timeInTrajectory);
         desiredPosition.set(trajectory.getPosition());

         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());
         lowLevelJointData.setDesiredVelocity(trajectory.getVelocity());
         lowLevelJointData.setDesiredAcceleration(trajectory.getAcceleration());
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onExit()
   {
      // Do nothing

   }

   @Override
   public boolean isDone(double timeInState)
   {
      return timeInState > (timeToMoveSittingDown.getDoubleValue() + minimumTimeDoneWithSittingDown.getDoubleValue());
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   private class TrajectoryData
   {
      private final DoubleProvider finalJointConfiguration;
      private final YoDouble desiredJointConfiguration;
      private final YoPolynomial jointTrajectory;

      public TrajectoryData(DoubleProvider finalJointConfiguration, YoDouble desiredJointConfiguration, YoPolynomial jointTrajectory)
      {
         this.finalJointConfiguration = finalJointConfiguration;
         this.desiredJointConfiguration = desiredJointConfiguration;
         this.jointTrajectory = jointTrajectory;
      }

      public DoubleProvider getFinalJointConfiguration()
      {
         return finalJointConfiguration;
      }

      public YoDouble getDesiredJointConfiguration()
      {
         return desiredJointConfiguration;
      }

      public YoPolynomial getJointTrajectory()
      {
         return jointTrajectory;
      }
   }
}
