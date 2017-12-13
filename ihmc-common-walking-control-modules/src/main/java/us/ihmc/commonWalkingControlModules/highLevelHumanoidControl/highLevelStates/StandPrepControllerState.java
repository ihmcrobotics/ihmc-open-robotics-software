package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.variable.YoDouble;

public class StandPrepControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.STAND_PREP_STATE;
   private static final double MINIMUM_TIME_DONE_WITH_STAND_PREP = 0.0;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJoint, TrajectoryData> jointsData = new PairList<>();

   private final YoDouble timeToPrepareForStanding = new YoDouble("timeToPrepareForStanding", registry);
   private final YoDouble minimumTimeDoneWithStandPrep = new YoDouble("minimumTimeDoneWithStandPrep", registry);
   private final JointDesiredOutputListReadOnly highLevelControlOutput;

   public StandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                   JointDesiredOutputListReadOnly highLevelControlOutput)
   {
      this(controllerToolbox, highLevelControllerParameters, highLevelControlOutput, MINIMUM_TIME_DONE_WITH_STAND_PREP);
   }

   public StandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                   JointDesiredOutputListReadOnly highLevelControlOutput, double minimumTimeDoneWithStandPrep)
   {
      super(controllerState, highLevelControllerParameters, controllerToolbox);
      this.highLevelControlOutput = highLevelControlOutput;

      this.timeToPrepareForStanding.set(highLevelControllerParameters.getTimeToMoveInStandPrep());
      this.minimumTimeDoneWithStandPrep.set(minimumTimeDoneWithStandPrep);

      WholeBodySetpointParameters standPrepParameters = highLevelControllerParameters.getStandPrepParameters();
      OneDoFJoint[] controlledJoints = ScrewTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJoint.class);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();

         YoPolynomial trajectory = new YoPolynomial(jointName + "_StandPrepTrajectory", 4, registry);
         YoDouble standPrepFinalConfiguration = new YoDouble(jointName + "_StandPrepFinalConfiguration", registry);
         YoDouble standPrepDesiredConfiguration = new YoDouble(jointName + "_StandPrepDesiredConfiguration", registry);

         standPrepFinalConfiguration.set(standPrepParameters.getSetpoint(jointName));

         TrajectoryData jointData = new TrajectoryData(standPrepFinalConfiguration, standPrepDesiredConfiguration, trajectory);
         jointsData.add(controlledJoint, jointData);
      }

   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJoint joint = jointsData.get(jointIndex).getLeft();
         TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();
         YoDouble standPrepFinal = trajectoryData.getFinalJointConfiguration();
         YoPolynomial trajectory = trajectoryData.getJointTrajectory();

         double desiredFinalPosition = standPrepFinal.getDoubleValue();
         double desiredFinalVelocity = 0.0;

         JointDesiredOutputReadOnly jointDesiredOutput = highLevelControlOutput.getJointDesiredOutput(joint);
         double startAngle;
         if (jointDesiredOutput != null && jointDesiredOutput.hasDesiredPosition())
            startAngle = jointDesiredOutput.getDesiredPosition();
         else
            startAngle = joint.getQ();
         double startVelocity = 0.0;

         trajectory.setCubic(0.0, timeToPrepareForStanding.getDoubleValue(), startAngle, startVelocity, desiredFinalPosition, desiredFinalVelocity);
      }
   }

   @Override
   public void doAction()
   {
      double timeInTrajectory = MathTools.clamp(getTimeInCurrentState(), 0.0, timeToPrepareForStanding.getDoubleValue());

      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJoint joint = jointsData.get(jointIndex).getLeft();
         TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

         YoPolynomial trajectory = trajectoryData.getJointTrajectory();
         YoDouble desiredPosition = trajectoryData.getDesiredJointConfiguration();

         trajectory.compute(timeInTrajectory);
         desiredPosition.set(trajectory.getPosition());

         JointDesiredOutput lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());
         lowLevelJointData.setDesiredVelocity(trajectory.getVelocity());
         lowLevelJointData.setDesiredAcceleration(trajectory.getAcceleration());
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
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
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   private class TrajectoryData
   {
      private final YoDouble finalJointConfiguration;
      private final YoDouble desiredJointConfiguration;
      private final YoPolynomial jointTrajectory;

      public TrajectoryData(YoDouble finalJointConfiguration, YoDouble desiredJointConfiguration, YoPolynomial jointTrajectory)
      {
         this.finalJointConfiguration = finalJointConfiguration;
         this.desiredJointConfiguration = desiredJointConfiguration;
         this.jointTrajectory = jointTrajectory;
      }

      public YoDouble getFinalJointConfiguration()
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
