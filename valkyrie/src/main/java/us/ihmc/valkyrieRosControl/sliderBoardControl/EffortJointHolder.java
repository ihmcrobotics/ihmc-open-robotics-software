package us.ihmc.valkyrieRosControl.sliderBoardControl;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
class EffortJointHolder extends ValkyrieSliderBoardJointHolder
{
   private final EffortJointHandle handle;

   public EffortJointHolder(ValkyrieRosControlSliderBoard valkyrieRosControlSliderBoard, OneDoFJointBasics joint, EffortJointHandle handle,
         YoRegistry parentRegistry, double dt)
   {
      super(valkyrieRosControlSliderBoard, joint, parentRegistry, dt);
      this.handle = handle;

      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {
      joint.setQ(handle.getPosition());
      joint.setQd(handle.getVelocity());
      bl_qd.update();
      joint.setTau(handle.getEffort());

      q.set(joint.getQ());
      qd.set(joint.getQd());
      tau.set(joint.getTau());

      double pdOutput = pdController.compute(q.getDoubleValue(), q_d.getDoubleValue(), qd.getDoubleValue(), qd_d.getDoubleValue());
      jointCommand_pd.set(pdOutput);
      tau_d.set(valkyrieRosControlSliderBoard.masterScaleFactor.getDoubleValue() * (jointCommand_pd.getDoubleValue() + jointCommand_function.getDoubleValue()) + tau_offset
            .getDoubleValue());

      handle.setDesiredEffort(tau_d.getDoubleValue());
   }
}
