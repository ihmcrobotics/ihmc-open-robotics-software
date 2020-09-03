package us.ihmc.valkyrieRosControl.sliderBoardControl;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.rosControl.wholeRobot.PositionJointHandle;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
class PositionJointHolder extends ValkyrieSliderBoardJointHolder
{
   private final PositionJointHandle handle;
   private final DeltaLimitedYoVariable positionStepSizeLimiter;

   public PositionJointHolder(ValkyrieRosControlSliderBoard valkyrieRosControlSliderBoard, OneDoFJointBasics joint, PositionJointHandle handle,
         YoRegistry parentRegistry, double dt)
   {
      super(valkyrieRosControlSliderBoard, joint, parentRegistry, dt);
      this.handle = handle;

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(handle.getName() + "PositionStepSizeLimiter", registry, 0.15);
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

      positionStepSizeLimiter.updateOutput(q.getDoubleValue(), q_d.getDoubleValue());
      handle.setDesiredPosition(positionStepSizeLimiter.getDoubleValue());
   }
}
