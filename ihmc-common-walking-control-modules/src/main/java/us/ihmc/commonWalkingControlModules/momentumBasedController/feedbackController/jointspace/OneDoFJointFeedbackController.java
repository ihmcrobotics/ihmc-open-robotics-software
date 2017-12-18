package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class OneDoFJointFeedbackController implements FeedbackControllerInterface
{
   private final JointspaceAccelerationCommand inverseDynamicsOutput = new JointspaceAccelerationCommand();
   private final JointspaceVelocityCommand inverseKinematicsOutput = new JointspaceVelocityCommand();

   private final OneDoFJoint joint;

   private final YoBoolean isEnabled;

   private final YoDouble qCurrent;
   private final YoDouble qDCurrent;

   private final YoDouble qDesired;
   private final YoDouble qDDesired;

   private final YoDouble qDFeedforward;
   private final YoDouble qDDFeedforward;

   private final YoDouble qError;
   private final YoDouble qDError;

   private final YoDouble maxFeedback;
   private final YoDouble maxFeedbackRate;

   private final YoDouble qDDFeedback;
   private final RateLimitedYoVariable qDDFeedbackRateLimited;
   private final YoDouble qDDDesired;
   private final YoDouble qDDAchieved;

   private final YoDouble qDFeedback;
   private final RateLimitedYoVariable qDFeedbackRateLimited;

   private final YoDouble kp;
   private final YoDouble kd;

   private final YoDouble weightForSolver;

   public OneDoFJointFeedbackController(OneDoFJoint joint, double dt, boolean inverseDynamicsEnabled, boolean inverseKinematicsEnabled,
                                        boolean virtualModelControlEnabled, YoVariableRegistry parentRegistry)
   {
      if (!inverseDynamicsEnabled && !inverseKinematicsEnabled && !virtualModelControlEnabled)
         throw new RuntimeException("Controller core is not properly setup, none of the control modes is enabled.");

      String jointName = joint.getName();
      YoVariableRegistry registry = new YoVariableRegistry(jointName + "PDController");

      this.joint = joint;
      isEnabled = new YoBoolean("control_enabled_" + jointName, registry);
      isEnabled.set(false);

      qCurrent = new YoDouble("q_" + jointName, registry);
      qDesired = new YoDouble("q_d_" + jointName, registry);
      qError = new YoDouble("q_err_" + jointName, registry);

      qDDesired = new YoDouble("qd_d_" + jointName, registry);

      maxFeedback = new YoDouble("max_fb_" + jointName, registry);
      maxFeedbackRate = new YoDouble("max_fb_rate_" + jointName, registry);

      kp = new YoDouble("kp_" + jointName, registry);

      if (inverseDynamicsEnabled || virtualModelControlEnabled)
      {
         qDCurrent = new YoDouble("qd_" + jointName, registry);
         qDError = new YoDouble("qd_err_" + jointName, registry);

         kd = new YoDouble("kd_" + jointName, registry);

         qDDFeedforward = new YoDouble("qdd_ff_" + jointName, registry);
         qDDFeedback = new YoDouble("qdd_fb_" + jointName, registry);
         qDDFeedbackRateLimited = new RateLimitedYoVariable("qdd_fb_rl_" + jointName, registry, maxFeedbackRate, qDDFeedback, dt);
         qDDDesired = new YoDouble("qdd_d_" + jointName, registry);
         qDDAchieved = new YoDouble("qdd_achieved_" + jointName, registry);
      }
      else
      {
         qDCurrent = null;
         qDError = null;

         kd = null;

         qDDFeedforward = null;
         qDDFeedback = null;
         qDDFeedbackRateLimited = null;
         qDDDesired = null;
         qDDAchieved = null;
      }

      if (inverseKinematicsEnabled)
      {
         qDFeedforward = new YoDouble("qd_ff_" + jointName, registry);

         qDFeedback = new YoDouble("qd_fb_" + jointName, registry);
         qDFeedbackRateLimited = new RateLimitedYoVariable("qd_fb_rl_" + jointName, registry, maxFeedbackRate, qDDFeedback, dt);
      }
      else
      {
         qDFeedforward = null;

         qDFeedback = null;
         qDFeedbackRateLimited = null;
      }

      weightForSolver = new YoDouble("weight_" + jointName, registry);
      weightForSolver.set(Double.POSITIVE_INFINITY);

      inverseDynamicsOutput.addJoint(joint, Double.NaN);
      inverseKinematicsOutput.addJoint(joint, Double.NaN);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      qDDFeedbackRateLimited.reset();
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   public void setDesireds(double q_d, double qd_d, double qdd_feedforward)
   {
      qDesired.set(q_d);
      qDDesired.set(qd_d);
      if (qDFeedforward != null)
         qDFeedforward.set(qd_d);
      if (qDDFeedforward != null)
         qDDFeedforward.set(qdd_feedforward);
   }

   public void setGains(PDGainsReadOnly gains)
   {
      kp.set(gains.getKp());
      kd.set(gains.getKd());
      maxFeedback.set(gains.getMaximumFeedback());
      maxFeedbackRate.set(gains.getMaximumFeedbackRate());
   }

   public void setWeightForSolver(double weightForSolver)
   {
      this.weightForSolver.set(weightForSolver);
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

      double qdd_fb = kp.getDoubleValue() * qError.getDoubleValue() + kd.getDoubleValue() * qDError.getDoubleValue();
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
      qDFeedback.set(qd_fb);
      qDFeedbackRateLimited.update();

      qDDesired.set(qDFeedforward.getDoubleValue() + qDFeedbackRateLimited.getDoubleValue());
      inverseKinematicsOutput.setOneDoFJointDesiredVelocity(0, qDDesired.getDoubleValue());
      inverseKinematicsOutput.setWeight(0, weightForSolver.getDoubleValue());
   }

   @Override
   public void computeVirtualModelControl()
   {
      computeInverseDynamics();
   }

   @Override
   public void computeAchievedAcceleration()
   {
      qDDAchieved.set(joint.getQddDesired());
   }

   public OneDoFJoint getJoint()
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
   public JointspaceAccelerationCommand getVirtualModelControlOutput()
   {
      return getInverseDynamicsOutput();
   }
}
