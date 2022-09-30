package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import com.google.common.base.CaseFormat;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class StandPrepControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.STAND_PREP_STATE;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJointBasics, TrajectoryData> jointsData = new PairList<>();

   private final YoPolynomial trajectory;

   private final YoBoolean reinitialize = new YoBoolean("standPrepReinitialize", registry);
   private final YoBoolean continuousUpdate = new YoBoolean("standPrepContinuousUpdate", registry);
   private final YoDouble splineStartTime = new YoDouble("standPrepSplineStartTime", registry);
   private final YoDouble timeToPrepareForStanding = new YoDouble("timeToPrepareForStanding", registry);
   private final YoDouble minimumTimeDoneWithStandPrep = new YoDouble("minimumTimeDoneWithStandPrep", registry);
   private final JointDesiredOutputListReadOnly highLevelControlOutput;

   private final DoubleProvider timeProvider;

   public StandPrepControllerState(OneDoFJointBasics[] controlledJoints,
                                   HighLevelControllerParameters highLevelControllerParameters,
                                   JointDesiredOutputListReadOnly highLevelControlOutput,
                                   DoubleProvider timeProvider)
   {
      super(controllerState, highLevelControllerParameters, controlledJoints);
      this.highLevelControlOutput = highLevelControlOutput;
      this.timeProvider = timeProvider;

      this.timeToPrepareForStanding.set(highLevelControllerParameters.getTimeToMoveInStandPrep());

      WholeBodySetpointParameters standPrepParameters = highLevelControllerParameters.getStandPrepParameters();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      trajectory = new YoPolynomial("StandPrepTrajectory", 4, registry);


      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();
         String namePrefix = CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, jointName);

//         YoPolynomial trajectory = new YoPolynomial(namePrefix + "StandPrepTrajectory", 4, registry);
         DoubleProvider standPrepInitialConfiguration = new DoubleParameter(namePrefix + "StandPrepPosition",
                                                                            registry,
                                                                            standPrepParameters.getSetpoint(jointName));
         DoubleProvider standPrepFinalConfiguration = new DoubleParameter(namePrefix + "StandPrepPosition",
                                                                          registry,
                                                                          standPrepParameters.getSetpoint(jointName));
         YoDouble standPrepDesiredConfiguration = new YoDouble(namePrefix + "StandPrepCurrentDesired", registry);

         TrajectoryData jointData = new TrajectoryData(standPrepInitialConfiguration, standPrepFinalConfiguration,
                                                       standPrepDesiredConfiguration, trajectory);
         jointsData.add(controlledJoint, jointData);
      }

   }

   public void setMinimumTimeDoneWithStandPrep(double minimumTimeDoneWithStandPrep)
   {
      this.minimumTimeDoneWithStandPrep.set(minimumTimeDoneWithStandPrep);
   }

   @Override
   public void onEntry()
   {
      continuousUpdate.set(false);
      reinitialize.set(false);
      if (timeProvider != null)
         initializeSplines(timeProvider.getValue());
      else
         initializeSplines(0.0);
   }

   public void initializeSplines(double startTime)
   {
      splineStartTime.set(startTime);

      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointsData.get(jointIndex).getLeft();
         TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();
         DoubleProvider standPrepFinal = trajectoryData.getFinalJointConfiguration();
         YoPolynomial trajectory = trajectoryData.getJointTrajectory();

         double desiredFinalPosition = standPrepFinal.getValue();
         double desiredFinalVelocity = 0.0;

         JointDesiredOutputReadOnly jointDesiredOutput = highLevelControlOutput.getJointDesiredOutput(joint);
         double startAngle;
         if (jointDesiredOutput != null && jointDesiredOutput.hasDesiredPosition())
            startAngle = jointDesiredOutput.getDesiredPosition();
         else
            startAngle = joint.getQ();
         double startVelocity = 0.0;

//         trajectory.setCubic(0.0, timeToPrepareForStanding.getDoubleValue(), startAngle, startVelocity, desiredFinalPosition, desiredFinalVelocity);
      }

      trajectory.setCubic(0.0, timeToPrepareForStanding.getDoubleValue(), 0, 1, 0, 0);
   }

   @Override
   public void doAction(double timeInState)
   {
      double time;
      if (timeProvider != null)
         time = timeProvider.getValue();
      else
         time = timeInState;

      if (continuousUpdate.getValue())
      {
         reinitialize.set(false);
         initializeSplines(splineStartTime.getValue());
      }
      else if (reinitialize.getValue())
      {
         reinitialize.set(false);
         initializeSplines(time);
      }

      double timeInTrajectory = MathTools.clamp(time - splineStartTime.getValue(), 0.0, timeToPrepareForStanding.getDoubleValue());

      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         //Set up alpha for eacah joint
         trajectory.compute(timeInTrajectory);
         double alphaPosition = trajectory.getValue();
         double alphaVelocity = trajectory.getVelocity();

         OneDoFJointBasics joint = jointsData.get(jointIndex).getLeft();
         TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

         YoPolynomial trajectory = trajectoryData.getJointTrajectory();
         YoDouble desiredPosition = trajectoryData.getDesiredJointConfiguration();

         trajectory.compute(timeInTrajectory);
         desiredPosition.set(trajectory.getValue());

         //Not sure if this is correct, because q0 and qf are what need to be saved per joint, but when drawing on the whiteboard
         //q0 and qf were the parameters for the alpha functions. So how can I use something I am trying to get??
         double q_initial = trajectoryData.getInitialJoinConfiguration().getValue();
         double q_final = trajectoryData.getFinalJointConfiguration().getValue();;

         double jointPosition = (1 - alphaPosition) * q_initial + alphaPosition * q_final;
         double jointVelocity = alphaVelocity * (q_final - q_initial);

         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());
         lowLevelJointData.setDesiredVelocity(trajectory.getVelocity());
         lowLevelJointData.setDesiredAcceleration(trajectory.getAcceleration());
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onExit(double timeInState)
   {
      // Do nothing

   }

   @Override
   public boolean isDone(double timeInState)
   {
      return timeInState > (timeToPrepareForStanding.getDoubleValue() + minimumTimeDoneWithStandPrep.getDoubleValue());
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolder getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   private class TrajectoryData
   {
      private final DoubleProvider initialJoinConfiguration;
      private final DoubleProvider finalJointConfiguration;
      private final YoDouble desiredJointConfiguration;
      private final YoPolynomial jointTrajectory;

      public TrajectoryData(DoubleProvider initialJointConfiguration, DoubleProvider finalJointConfiguration, YoDouble desiredJointConfiguration, YoPolynomial jointTrajectory)
      {
         this.initialJoinConfiguration = initialJointConfiguration;
         this.finalJointConfiguration = finalJointConfiguration;
         this.desiredJointConfiguration = desiredJointConfiguration;
         this.jointTrajectory = jointTrajectory;
      }

      public DoubleProvider getInitialJoinConfiguration()
      {
         return initialJoinConfiguration;
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
