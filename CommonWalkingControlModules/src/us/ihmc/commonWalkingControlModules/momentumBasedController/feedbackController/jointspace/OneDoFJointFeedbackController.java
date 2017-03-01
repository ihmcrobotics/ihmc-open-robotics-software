package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class OneDoFJointFeedbackController implements FeedbackControllerInterface
{
   private final JointspaceAccelerationCommand output = new JointspaceAccelerationCommand();

   private final OneDoFJoint joint;

   private final BooleanYoVariable isEnabled;

   private final DoubleYoVariable qCurrent;
   private final DoubleYoVariable qDCurrent;

   private final DoubleYoVariable qDesired;
   private final DoubleYoVariable qDDesired;
   private final DoubleYoVariable qDDFeedforward;

   private final DoubleYoVariable qError;
   private final DoubleYoVariable qDError;

   private final DoubleYoVariable maxFeedbackAcceleration;
   private final DoubleYoVariable maxFeedbackJerk;

   private final DoubleYoVariable qDDFeedback;
   private final RateLimitedYoVariable qDDFeedbackRateLimited;
   private final DoubleYoVariable qDDDesired;
   private final DoubleYoVariable qDDAchieved;

   private final DoubleYoVariable kp;
   private final DoubleYoVariable kd;

   private final DoubleYoVariable weightForSolver;

   public OneDoFJointFeedbackController(OneDoFJoint joint, double dt, YoVariableRegistry parentRegistry)
   {
      String jointName = joint.getName();
      YoVariableRegistry registry = new YoVariableRegistry(jointName + "PDController");

      this.joint = joint;
      isEnabled = new BooleanYoVariable("control_enabled_" + jointName, registry);
      isEnabled.set(false);

      qCurrent = new DoubleYoVariable("q_" + jointName, registry);
      qDCurrent = new DoubleYoVariable("qd_" + jointName, registry);

      qDesired = new DoubleYoVariable("q_d_" + jointName, registry);
      qDDesired = new DoubleYoVariable("qd_d_" + jointName, registry);
      qDDFeedforward = new DoubleYoVariable("qdd_ff_" + jointName, registry);

      qError = new DoubleYoVariable("q_err_" + jointName, registry);
      qDError = new DoubleYoVariable("qd_err_" + jointName, registry);

      maxFeedbackAcceleration = new DoubleYoVariable("qdd_max_fb_" + jointName, registry);
      maxFeedbackJerk = new DoubleYoVariable("qddd_fb_max_" + jointName, registry);
      qDDFeedback = new DoubleYoVariable("qdd_fb_" + jointName, registry);
      qDDFeedbackRateLimited = new RateLimitedYoVariable("qdd_fb_rl_" + jointName, registry, maxFeedbackJerk, qDDFeedback, dt);
      qDDDesired = new DoubleYoVariable("qdd_d_" + jointName, registry);
      qDDAchieved = new DoubleYoVariable("qdd_achieved_" + jointName, registry);

      kp = new DoubleYoVariable("kp_" + jointName, registry);
      kd = new DoubleYoVariable("kd_" + jointName, registry);

      weightForSolver = new DoubleYoVariable("weight_" + jointName, registry);
      weightForSolver.set(Double.POSITIVE_INFINITY);

      output.addJoint(joint, Double.NaN);

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
      qDDFeedforward.set(qdd_feedforward);
   }

   public void setGains(PDGainsInterface gains)
   {
      kp.set(gains.getKp());
      kd.set(gains.getKd());
      maxFeedbackAcceleration.set(gains.getMaximumFeedback());
      maxFeedbackJerk.set(gains.getMaximumFeedbackRate());
   }

   public void setWeightForSolver(double weightForSolver)
   {
      this.weightForSolver.set(weightForSolver);
      output.setWeight(weightForSolver);
   }

   @Override
   public void compute()
   {
      if (!isEnabled.getBooleanValue())
         return;

      qCurrent.set(joint.getQ());
      qDCurrent.set(joint.getQd());

      qError.set(qDesired.getDoubleValue() - qCurrent.getDoubleValue());
      qDError.set(qDDesired.getDoubleValue() - qDCurrent.getDoubleValue());

      double qdd_fb = kp.getDoubleValue() * qError.getDoubleValue() + kd.getDoubleValue() * qDError.getDoubleValue();
      qdd_fb = MathTools.clamp(qdd_fb, maxFeedbackAcceleration.getDoubleValue());
      qDDFeedback.set(qdd_fb);
      qDDFeedbackRateLimited.update();

      qDDDesired.set(qDDFeedforward.getDoubleValue() + qDDFeedbackRateLimited.getDoubleValue());
      output.setOneDoFJointDesiredAcceleration(0, qDDDesired.getDoubleValue());
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
   public JointspaceAccelerationCommand getOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return output;
   }
}
