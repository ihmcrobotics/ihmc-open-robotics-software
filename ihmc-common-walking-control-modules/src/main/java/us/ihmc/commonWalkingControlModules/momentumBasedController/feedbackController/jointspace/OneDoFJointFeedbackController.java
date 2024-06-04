package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings.FilterDouble1D;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class OneDoFJointFeedbackController implements FeedbackControllerInterface
{
   private final JointspaceAccelerationCommand inverseDynamicsOutput = new JointspaceAccelerationCommand();
   private final JointspaceVelocityCommand inverseKinematicsOutput = new JointspaceVelocityCommand();
   private final JointTorqueCommand virtualModelControlOutput = new JointTorqueCommand();

   private final OneDoFJointBasics joint;

   private final YoBoolean isEnabled;

   private final YoDouble qCurrent;
   private final YoDouble qDCurrent;

   private final YoDouble qDesired;
   private final YoDouble qDDesired;

   private final YoDouble qDFeedforward;
   private final YoDouble qDDFeedforward;
   private final YoDouble tauFeedforward;

   private final YoDouble qError;
   private final YoDouble qDError;
   private final AlphaFilteredYoVariable qDFilteredError;
   private final FilterDouble1D qDErrorFilter;

   private final YoDouble actionP;
   private final YoDouble actionD;

   private final YoDouble maxFeedback;
   private final YoDouble maxFeedbackRate;

   private final YoDouble qDDFeedback;
   private final RateLimitedYoVariable qDDFeedbackRateLimited;
   private final YoDouble qDDDesired;
   private final YoDouble qDDAchieved;

   private final YoDouble tauFeedback;
   private final RateLimitedYoVariable tauFeedbackRateLimited;
   private final YoDouble tauDesired;

   private final YoDouble qDFeedback;
   private final RateLimitedYoVariable qDFeedbackRateLimited;

   private final YoDouble kp;
   private final YoDouble kd;

   private final YoDouble weightForSolver;

   public OneDoFJointFeedbackController(OneDoFJointBasics joint,
                                        WholeBodyControlCoreToolbox toolbox,
                                        FeedbackControllerToolbox feedbackControllerToolbox,
                                        YoRegistry parentRegistry)
   {
      String jointName = joint.getName();
      YoRegistry registry = feedbackControllerToolbox.getRegistry();

      this.joint = joint;
      isEnabled = new YoBoolean("control_enabled_" + jointName, registry);
      isEnabled.set(false);

      double dt = toolbox.getControlDT();

      YoRegistry debugVariableRegistry = WholeBodyControllerCore.REDUCE_YOVARIABLES ? null : registry;

      qCurrent = new YoDouble("q_" + jointName, debugVariableRegistry);
      qDesired = new YoDouble("q_d_" + jointName, registry);
      qError = new YoDouble("q_err_" + jointName, debugVariableRegistry);

      actionP = new YoDouble("action_P_" + jointName, debugVariableRegistry);
      actionD = new YoDouble("action_D_" + jointName, debugVariableRegistry);

      qDDesired = new YoDouble("qd_d_" + jointName, registry);

      maxFeedback = new YoDouble("max_fb_" + jointName, debugVariableRegistry);
      maxFeedbackRate = new YoDouble("max_fb_rate_" + jointName, debugVariableRegistry);

      kp = new YoDouble("kp_" + jointName, registry);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {
         qDCurrent = new YoDouble("qd_" + jointName, debugVariableRegistry);
         qDError = new YoDouble("qd_err_" + jointName, debugVariableRegistry);

         DoubleProvider breakFrequency = feedbackControllerToolbox.getErrorVelocityFilterBreakFrequency(jointName);
         if (breakFrequency != null)
         {
            DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency.getValue(), dt);
            qDFilteredError = new AlphaFilteredYoVariable(jointName, debugVariableRegistry, alpha, qDError);
         }
         else
         {
            qDFilteredError = null;
         }

         qDErrorFilter = feedbackControllerToolbox.getOrCreateVelocityErrorFilterDouble1D(jointName, dt);

         kd = new YoDouble("kd_" + jointName, registry);

         qDDFeedforward = new YoDouble("qdd_ff_" + jointName, debugVariableRegistry);
         qDDFeedback = new YoDouble("qdd_fb_" + jointName, debugVariableRegistry);
         qDDFeedbackRateLimited = new RateLimitedYoVariable("qdd_fb_rl_" + jointName, debugVariableRegistry, maxFeedbackRate, qDDFeedback, dt);
         qDDDesired = new YoDouble("qdd_d_" + jointName, registry);
         qDDAchieved = new YoDouble("qdd_achieved_" + jointName, debugVariableRegistry);
      }
      else
      {
         qDCurrent = null;
         qDError = null;
         qDFilteredError = null;
         qDErrorFilter = null;

         kd = null;

         qDDFeedforward = null;
         qDDFeedback = null;
         qDDFeedbackRateLimited = null;
         qDDDesired = null;
         qDDAchieved = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         qDFeedforward = new YoDouble("qd_ff_" + jointName, debugVariableRegistry);

         qDFeedback = new YoDouble("qd_fb_" + jointName, debugVariableRegistry);
         qDFeedbackRateLimited = new RateLimitedYoVariable("qd_fb_rl_" + jointName, debugVariableRegistry, maxFeedbackRate, qDFeedback, dt);
      }
      else
      {
         qDFeedforward = null;

         qDFeedback = null;
         qDFeedbackRateLimited = null;
      }

      if (toolbox.isEnableVirtualModelControlModule())
      {
         tauFeedback = new YoDouble("tau_fb_" + jointName, debugVariableRegistry);
         tauFeedforward = new YoDouble("tau_ff_" + jointName, debugVariableRegistry);
         tauFeedbackRateLimited = new RateLimitedYoVariable("tau_fb_rl_" + jointName, debugVariableRegistry, maxFeedbackRate, tauFeedback, dt);

         tauDesired = new YoDouble("tau_d_" + jointName, registry);
      }
      else
      {
         tauFeedback = null;
         tauFeedforward = null;
         tauFeedbackRateLimited = null;

         tauDesired = null;
      }

      weightForSolver = new YoDouble("weight_" + jointName, debugVariableRegistry);
      weightForSolver.set(Double.POSITIVE_INFINITY);

      inverseDynamicsOutput.addJoint(joint, Double.NaN);
      inverseKinematicsOutput.addJoint(joint, Double.NaN);
      virtualModelControlOutput.addJoint(joint, Double.NaN);
   }

   @Override
   public void initialize()
   {
      if (qDDFeedbackRateLimited != null)
         qDDFeedbackRateLimited.reset();
      if (qDFilteredError != null)
         qDFilteredError.reset();
      if (qDErrorFilter != null)
         qDErrorFilter.reset();
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   public void submitFeedbackControlCommand(OneDoFJointFeedbackControlCommand command)
   {

      weightForSolver.set(command.getWeightForSolver());

      PDGainsReadOnly gains = command.getGains();
      kp.set(gains.getKp());
      if (kd != null)
         kd.set(gains.getKd());
      maxFeedback.set(gains.getMaximumFeedback());
      maxFeedbackRate.set(gains.getMaximumFeedbackRate());

      qDesired.set(command.getReferencePosition());
      qDDesired.set(command.getReferenceVelocity());

      if (qDFeedforward != null)
         qDFeedforward.set(command.getReferenceVelocity());
      if (qDDFeedforward != null)
         qDDFeedforward.set(command.getReferenceAcceleration());
      if (tauFeedforward != null)
         tauFeedforward.set(command.getReferenceEffort());
   }

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled.getBooleanValue())
         return;

      qCurrent.set(joint.getQ());
      qDCurrent.set(joint.getQd());

      qError.set(qDesired.getDoubleValue() - qCurrent.getDoubleValue());
      qDError.set(qDDesired.getDoubleValue() - qDCurrent.getDoubleValue());
      double qDErrorToUse = qDError.getDoubleValue();
      if (qDFilteredError != null)
      {
         qDFilteredError.update();
         qDErrorToUse = qDFilteredError.getValue();
      }
      if (qDErrorFilter != null)
         qDErrorToUse = qDErrorFilter.apply(qDErrorToUse);

      actionP.set(kp.getDoubleValue() * qError.getDoubleValue());
      actionD.set(kd.getDoubleValue() * qDErrorToUse);

      double qdd_fb = actionP.getDoubleValue() + actionD.getValue();
      qdd_fb = MathTools.clamp(qdd_fb, maxFeedback.getDoubleValue());
      qDDFeedback.set(qdd_fb);
      qDDFeedbackRateLimited.update();

      qDDDesired.set(qDDFeedforward.getDoubleValue() + qDDFeedbackRateLimited.getDoubleValue());
      inverseDynamicsOutput.setOneDoFJointDesiredAcceleration(0, qDDDesired.getDoubleValue());
      inverseDynamicsOutput.setWeight(0, weightForSolver.getDoubleValue());
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled.getBooleanValue())
         return;

      qCurrent.set(joint.getQ());

      qError.set(qDesired.getDoubleValue() - qCurrent.getDoubleValue());

      double qd_fb = kp.getDoubleValue() * qError.getDoubleValue();
      qd_fb = MathTools.clamp(qd_fb, maxFeedback.getDoubleValue());

      actionP.set(qd_fb);
      actionD.set(0.0);

      qDFeedback.set(qd_fb);
      qDFeedbackRateLimited.update();

      qDDesired.set(qDFeedforward.getDoubleValue() + qDFeedbackRateLimited.getDoubleValue());
      inverseKinematicsOutput.setOneDoFJointDesiredVelocity(0, qDDesired.getDoubleValue());
      inverseKinematicsOutput.setWeight(0, weightForSolver.getDoubleValue());
   }

   @Override
   public void computeVirtualModelControl()
   {
      if (!isEnabled.getBooleanValue())
         return;

      qCurrent.set(joint.getQ());
      qDCurrent.set(joint.getQd());

      qError.set(qDesired.getDoubleValue() - qCurrent.getDoubleValue());
      qDError.set(qDDesired.getDoubleValue() - qDCurrent.getDoubleValue());
      double qDErrorToUse = qDError.getDoubleValue();
      if (qDFilteredError != null)
      {
         qDFilteredError.update();
         qDErrorToUse = qDFilteredError.getValue();
      }

      actionP.set(kp.getDoubleValue() * qError.getDoubleValue());
      actionD.set(kd.getDoubleValue() * qDErrorToUse);
      double tau_fb = actionP.getDoubleValue() + actionD.getDoubleValue();

      tau_fb = MathTools.clamp(tau_fb, maxFeedback.getDoubleValue());
      tauFeedback.set(tau_fb);
      tauFeedbackRateLimited.update();

      tauDesired.set(tauFeedforward.getValue() + tauFeedbackRateLimited.getDoubleValue());

      virtualModelControlOutput.setOneDoFJointDesiredTorque(0, tauDesired.getDoubleValue());
   }

   @Override
   public void computeAchievedAcceleration()
   {
      qDDAchieved.set(joint.getQdd());
   }

   public OneDoFJointBasics getJoint()
   {
      return joint;
   }

   public double getJointDesiredAcceleration()
   {
      return qDDDesired.getDoubleValue();
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   @Override
   public JointspaceAccelerationCommand getInverseDynamicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseDynamicsOutput;
   }

   @Override
   public JointspaceVelocityCommand getInverseKinematicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseKinematicsOutput;
   }

   @Override
   public JointTorqueCommand getVirtualModelControlOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return virtualModelControlOutput;
   }
}
