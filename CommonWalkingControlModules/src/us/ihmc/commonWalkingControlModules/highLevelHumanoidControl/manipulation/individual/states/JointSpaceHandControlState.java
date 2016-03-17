package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.LinkedHashMap;
import java.util.Map;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.trajectories.DoubleTrajectoryGenerator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointSpaceHandControlState extends HandControlState
{
   private final OneDoFJoint[] oneDoFJoints;
   private Map<OneDoFJoint, ? extends DoubleTrajectoryGenerator> trajectories;
   private final LinkedHashMap<OneDoFJoint, PIDController> pidControllers;
   private final JointspaceFeedbackControlCommand jointspaceFeedbackControlCommand = new JointspaceFeedbackControlCommand();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelJointDesiredData;

   private final LinkedHashMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedAccelerations;

   private final DoubleYoVariable maxAcceleration;

   private final BooleanYoVariable setDesiredJointAccelerations;

   private final YoVariableRegistry registry;
   private final BooleanYoVariable initialized;

   private final boolean doPositionControl;

   private final double dt;
   private final boolean[] doIntegrateDesiredAccelerations;

   public JointSpaceHandControlState(String namePrefix, OneDoFJoint[] controlledJoints, boolean doPositionControl,
         MomentumBasedController momentumBasedController, YoPIDGains gains, double dt, YoVariableRegistry parentRegistry)
   {
      super(HandControlMode.JOINT_SPACE);

      this.dt = dt;
      this.doPositionControl = doPositionControl;

      String name = namePrefix + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);

      oneDoFJoints = controlledJoints;
      initialized = new BooleanYoVariable(name + "Initialized", registry);
      initialized.set(false);

      if (!doPositionControl)
      {
         lowLevelJointDesiredData = null;
         setDesiredJointAccelerations = null;

         maxAcceleration = gains.getYoMaximumAcceleration();
         pidControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
         rateLimitedAccelerations = new LinkedHashMap<OneDoFJoint, RateLimitedYoVariable>();

         for (OneDoFJoint joint : oneDoFJoints)
         {
            String suffix = StringUtils.uncapitalize(joint.getName());
            PIDController pidController = new PIDController(gains.getYoKp(), gains.getYoKi(), gains.getYoKd(), gains.getYoMaxIntegralError(), suffix, registry);
            pidControllers.put(joint, pidController);

            RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(suffix + "FeedbackAcceleration", registry, gains.getYoMaximumJerk(), dt);
            rateLimitedAccelerations.put(joint, rateLimitedAcceleration);
         }
      }
      else
      {
         lowLevelJointDesiredData = new LowLevelOneDoFJointDesiredDataHolder(oneDoFJoints.length);
         lowLevelJointDesiredData.registerJointsWithEmptyData(oneDoFJoints);
         lowLevelJointDesiredData.setJointsControlMode(oneDoFJoints, LowLevelJointControlMode.POSITION_CONTROL);

         setDesiredJointAccelerations = new BooleanYoVariable(namePrefix + "SetDesiredJointAccelerations", registry);
         setDesiredJointAccelerations.set(false);

         maxAcceleration = null;
         pidControllers = null;
         rateLimitedAccelerations = null;
      }

      doIntegrateDesiredAccelerations = new boolean[oneDoFJoints.length];

      jointspaceFeedbackControlCommand.setWeightForSolver(SolverWeightLevels.ARM_JOINTSPACE_WEIGHT);
      jointspaceFeedbackControlCommand.setGains(gains);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         doIntegrateDesiredAccelerations[i] = joint.getIntegrateDesiredAccelerations();
         jointspaceFeedbackControlCommand.addJoint(joint, Double.NaN, Double.NaN, Double.NaN);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeight(double weight)
   {
      jointspaceFeedbackControlCommand.setWeightForSolver(weight);
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];

         DoubleTrajectoryGenerator trajectoryGenerator = trajectories.get(joint);
         trajectoryGenerator.compute(getTimeInCurrentState());

         double desiredPosition = trajectoryGenerator.getValue();
         joint.setqDesired(desiredPosition);
         double desiredVelocity = trajectoryGenerator.getVelocity();
         joint.setQdDesired(desiredVelocity);
         double feedForwardAcceleration = trajectoryGenerator.getAcceleration();

         if (doPositionControl)
         {
            enablePositionControl();
            if (!setDesiredJointAccelerations.getBooleanValue())
               feedForwardAcceleration = 0.0;
         }
         else
         {
            double currentPosition = joint.getQ();
            double currentVelocity = joint.getQd();

            PIDController pidController = pidControllers.get(joint);
            double desiredAcceleration = pidController.computeForAngles(currentPosition, joint.getqDesired(), currentVelocity, joint.getQdDesired(), dt);

            desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, maxAcceleration.getDoubleValue());

            RateLimitedYoVariable rateLimitedAcceleration = rateLimitedAccelerations.get(joint);
            rateLimitedAcceleration.update(desiredAcceleration);
            desiredAcceleration = rateLimitedAcceleration.getDoubleValue();

            jointspaceFeedbackControlCommand.setOneDoFJoint(i, desiredPosition, desiredVelocity, feedForwardAcceleration);
         }
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (!initialized.getBooleanValue() || getPreviousState() != this)
      {
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];

            if (!doPositionControl)
               pidControllers.get(joint).setCumulativeError(0.0);
         }
         initialized.set(true);
      }

      saveDoAccelerationIntegration();

      if (doPositionControl)
         enablePositionControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      disablePositionControl();
   }

   private void saveDoAccelerationIntegration()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         doIntegrateDesiredAccelerations[i] = joint.getIntegrateDesiredAccelerations();
      }
   }

   private void enablePositionControl()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setIntegrateDesiredAccelerations(false);
         joint.setUnderPositionControl(true);
      }
   }

   private void disablePositionControl()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setIntegrateDesiredAccelerations(doIntegrateDesiredAccelerations[i]);
         joint.setUnderPositionControl(false);
      }
   }

   @Override
   public boolean isDone()
   {
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         if (!trajectories.get(oneDoFJoint).isDone())
            return false;
      }

      return true;
   }

   public void setTrajectories(Map<OneDoFJoint, ? extends DoubleTrajectoryGenerator> trajectories)
   {
      this.trajectories = trajectories;
   }

   @Override
   public JointspaceAccelerationCommand getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public JointspaceFeedbackControlCommand getFeedbackControlCommand()
   {
      return jointspaceFeedbackControlCommand;
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolderInterface getLowLevelJointDesiredData()
   {
      return lowLevelJointDesiredData;
   }
}
