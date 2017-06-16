package us.ihmc.valkyrieRosControl.sliderBoardControl;

import us.ihmc.robotics.controllers.PDController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.RevisedBacklashCompensatingVelocityYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
abstract class ValkyrieSliderBoardJointHolder
{
   protected final ValkyrieRosControlSliderBoard valkyrieRosControlSliderBoard;
   protected final OneDoFJoint joint;
   protected final double dt;
   protected final YoVariableRegistry registry;
   protected final PDController pdController;
   protected final YoDouble q;
   protected final YoDouble qd;
   protected final YoDouble tau;
   protected final RevisedBacklashCompensatingVelocityYoVariable bl_qd;
   protected final YoDouble q_d;
   protected final YoDouble qd_d;
   protected final YoDouble tau_offset;
   protected final YoDouble jointCommand_pd;
   protected final YoDouble jointCommand_function;
   protected final YoDouble tau_d;

   public ValkyrieSliderBoardJointHolder(ValkyrieRosControlSliderBoard valkyrieRosControlSliderBoard, OneDoFJoint joint, YoVariableRegistry parentRegistry, double dt)
   {
      this.valkyrieRosControlSliderBoard = valkyrieRosControlSliderBoard;
      this.joint = joint;
      this.dt = dt;

      String jointName = joint.getName();
      this.registry = new YoVariableRegistry(jointName);
      this.pdController = new PDController(jointName, registry);
      pdController.setProportionalGain(ValkyrieRosControlSliderBoard.KP_DEFAULT);
      pdController.setDerivativeGain(ValkyrieRosControlSliderBoard.KD_DEFAULT);

      q = new YoDouble(jointName + "_q", registry);
      qd = new YoDouble(jointName + "_qd", registry);
      bl_qd = new RevisedBacklashCompensatingVelocityYoVariable("bl_qd_" + jointName, "", valkyrieRosControlSliderBoard.jointVelocityAlphaFilter, q, dt, valkyrieRosControlSliderBoard.jointVelocitySlopTime, registry);
      tau = new YoDouble(jointName + "_tau", registry);

      q_d = new YoDouble(jointName + "_q_d", registry);
      qd_d = new YoDouble(jointName + "_qd_d", registry);

      if (valkyrieRosControlSliderBoard.setPointMap != null && valkyrieRosControlSliderBoard.setPointMap.containsKey(jointName))
         q_d.set(valkyrieRosControlSliderBoard.setPointMap.get(jointName));

      tau_offset = new YoDouble(joint.getName() + "_tau_offset", parentRegistry);
      tau_d = new YoDouble(joint.getName() + "_tau_d", registry);
      jointCommand_pd = new YoDouble(joint.getName() + "_tau_pd", registry);
      jointCommand_function = new YoDouble(joint.getName() + "_tau_function", registry);

      if (valkyrieRosControlSliderBoard.torqueOffsetMap != null && valkyrieRosControlSliderBoard.torqueOffsetMap.containsKey(joint.getName()))
         tau_offset.set(-valkyrieRosControlSliderBoard.torqueOffsetMap.get(joint.getName()));
   }

   abstract void update();
}
