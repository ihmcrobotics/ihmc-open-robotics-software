package us.ihmc.quadrupedRobotics.controller.states;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * A controller that will track the minimum jerk trajectory to bring joints to a preparatory pose.
 */
public class QuadrupedStandPrepController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

//   private final DoubleParameter trajectoryTimeParameter = new DoubleParameter("trajectoryTime", registry, 1.0);
   private final double trajectoryTime = 5.0;
   private final QuadrupedInitialPositionParameters initialPositionParameters;

   private final FullQuadrupedRobotModel fullRobotModel;
   private final double dt;

   private final List<MinimumJerkTrajectory> trajectories;
   private final JointDesiredOutputList jointDesiredOutputList;

   private final YoDouble standPrepJointStiffnessSetpoint = new YoDouble("standPrepJointStiffnessSetpoint", registry);
   private final RateLimitedYoVariable standPrepJointStiffness = new RateLimitedYoVariable("standPrepJointStiffness", registry, 50.0, standPrepJointStiffnessSetpoint, 0.01);
   private final YoDouble standPrepJointDamping = new YoDouble("standPrepJointDamping", registry);
   private final YoDouble standPrepMasterGainSetpoint = new YoDouble("standPrepMasterGainSetpoint", registry);
   private final RateLimitedYoVariable standPrepMasterGain = new RateLimitedYoVariable("standPrepMasterGain", registry, 1.0 / 5.0, standPrepMasterGainSetpoint, 0.01);
   private final double[] previousPositionJointAngles = new double[12];

   private final YoDouble standPrepPositionMasterGainSetpoint = new YoDouble("standPrepPositionMasterGainSetpoint", registry);
   private final RateLimitedYoVariable standPrepPositionMasterGain = new RateLimitedYoVariable("standPrepPositionMasterGain", registry, 1.0 / 5.0, standPrepPositionMasterGainSetpoint, 0.01);

   private final YoBoolean yoUseForceFeedbackControl;
   private final YoBoolean goToPreviousPosition = new YoBoolean("goToPreviousPosition", registry);

   private final HashMap<OneDoFJoint, YoDouble> standPrepEffortMap = new HashMap<>();
   private final HashMap<OneDoFJoint, PDController> controllerMap = new HashMap<>();

   /**
    * The time from the beginning of the current preparation trajectory in seconds.
    */
   private double timeInTrajectory = 0.0;

   public QuadrupedStandPrepController(QuadrupedRuntimeEnvironment environment, QuadrupedInitialPositionParameters initialPositionParameters,
                                       QuadrupedControlMode controlMode, YoVariableRegistry parentRegistry)
   {
      this.initialPositionParameters = initialPositionParameters;
      this.fullRobotModel = environment.getFullRobotModel();
      this.jointDesiredOutputList = environment.getJointDesiredOutputList();
      this.dt = environment.getControlDT();

      standPrepJointStiffnessSetpoint.set(400.0);
      standPrepJointStiffness.set(400.0);
      standPrepJointDamping.set(25.0);

      standPrepPositionMasterGainSetpoint.set(1.0);
      standPrepPositionMasterGain.set(1.0);

      this.trajectories = new ArrayList<>(fullRobotModel.getOneDoFJoints().length);
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         standPrepEffortMap.put(joint, new YoDouble(joint.getName() + "StandPrepEffort", registry));
         trajectories.add(new MinimumJerkTrajectory());
         controllerMap.put(joint, new PDController(standPrepJointStiffness, standPrepJointDamping,joint.getName() + "JointController", registry));
      }

      yoUseForceFeedbackControl = new YoBoolean("useForceControlStandPrep", registry);
      yoUseForceFeedbackControl.set(controlMode == QuadrupedControlMode.FORCE);
      goToPreviousPosition.set(false);

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);

         QuadrupedJointName jointId = fullRobotModel.getNameForOneDoFJoint(joint);
         double desiredPosition = initialPositionParameters.getInitialJointPosition(jointId);

         // Start the trajectory from the current pos/vel/acc.
         MinimumJerkTrajectory trajectory = trajectories.get(i);

         double initialPosition = joint.getQ();
         double initialVelocity = 0.0;
         double initialAcceleration = 0.0;

         desiredPosition = initialPosition + AngleTools.trimAngleMinusPiToPi(desiredPosition - initialPosition);
         trajectory.setMoveParameters(initialPosition, 0.0, 0.0, desiredPosition, 0.0, 0.0, trajectoryTime);

         jointDesiredOutput.clear();
         jointDesiredOutput.setControlMode(JointDesiredControlMode.POSITION);
         previousPositionJointAngles[i] = initialPosition;
      }

      // This is a new trajectory. We start at time 0.
      timeInTrajectory = 0.0;
   }

   @Override
   public void doAction(double timeInState)
   {
      if(goToPreviousPosition.getBooleanValue())
      {
         for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
         {
            OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
            double qDesiredCurrent = jointDesiredOutputList.getJointDesiredOutput(joint).getDesiredPosition();
            trajectories.get(i).setMoveParameters(qDesiredCurrent, 0.0, 0.0, previousPositionJointAngles[i], 0.0, 0.0, trajectoryTime);
         }

         timeInTrajectory = 0.0;
         goToPreviousPosition.set(false);
      }

      standPrepJointStiffness.update();
      standPrepMasterGain.update();
      standPrepPositionMasterGain.update();
      fullRobotModel.updateFrames();

      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         MinimumJerkTrajectory trajectory = trajectories.get(i);
         PDController jointController = controllerMap.get(joint);

         double q = joint.getQ();
         double qd = joint.getQd();

         trajectory.computeTrajectory(timeInTrajectory);
         double qDesired = trajectory.getPosition();
         double qdDesired = trajectory.getVelocity();

         double effortDesired = jointController.compute(q, qDesired, qd, qdDesired);
         effortDesired *= standPrepMasterGain.getValue();
         effortDesired = MathTools.clamp(effortDesired, 100.0);

         standPrepEffortMap.get(joint).set(effortDesired);
         jointDesiredOutputList.getJointDesiredOutput(joint).setDesiredTorque(effortDesired);

         qDesired = q + standPrepPositionMasterGain.getDoubleValue() * (qDesired - q);
         qdDesired = qd + standPrepPositionMasterGain.getDoubleValue() * (qdDesired - qd);

         jointDesiredOutputList.getJointDesiredOutput(joint).setDesiredPosition(qDesired);
         jointDesiredOutputList.getJointDesiredOutput(joint).setDesiredVelocity(qdDesired);

         jointDesiredOutputList.getJointDesiredOutput(joint).setStiffness(standPrepJointStiffness.getValue());
         jointDesiredOutputList.getJointDesiredOutput(joint).setDamping(standPrepJointDamping.getValue());

      }

      timeInTrajectory += dt;
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      return isMotionExpired() ? ControllerEvent.DONE : null;
   }

   @Override
   public void onExit()
   {
   }

   private boolean isMotionExpired()
   {
      return timeInTrajectory > trajectoryTime;
   }
}

