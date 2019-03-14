package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class LinearMomentumRateControlModuleOutput
{
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final CenterOfPressureCommand centerOfPressureCommand = new CenterOfPressureCommand();

   private final FramePoint2DBasics desiredCMP = new FramePoint2D();

   private final FrameVector3D effectiveICPAdjustment = new FrameVector3D();
   private boolean usingStepAdjustment;
   private boolean footstepWasAdjusted;
   private final FramePose3D footstepSolution = new FramePose3D();

   public void setFootstepSolution(FramePose3DReadOnly footstepSolution)
   {
      this.footstepSolution.setIncludingFrame(footstepSolution);
   }

   public FramePose3DReadOnly getFootstepSolution()
   {
      return footstepSolution;
   }

   public void setFootstepWasAdjusted(boolean footstepWasAdjusted)
   {
      this.footstepWasAdjusted = footstepWasAdjusted;
   }

   public boolean getFootstepWasAdjusted()
   {
      return footstepWasAdjusted;
   }

   public void setUsingStepAdjustment(boolean usingStepAdjustment)
   {
      this.usingStepAdjustment = usingStepAdjustment;
   }

   public boolean getUsingStepAdjustment()
   {
      return usingStepAdjustment;
   }

   public void setMomentumRateCommand(MomentumRateCommand momentumRateCommand)
   {
      this.momentumRateCommand.set(momentumRateCommand);
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }

   public void setCenterOfPressureCommand(CenterOfPressureCommand centerOfPressureCommand)
   {
      this.centerOfPressureCommand.set(centerOfPressureCommand);
   }

   public CenterOfPressureCommand getCenterOfPressureCommand()
   {
      return centerOfPressureCommand;
   }

   public void setDesiredCMP(FramePoint2DReadOnly desiredCMP)
   {
      this.desiredCMP.setIncludingFrame(desiredCMP);
   }

   public FramePoint2DReadOnly getDesiredCMP()
   {
      return desiredCMP;
   }

   public void setEffectiveICPAdjustment(FrameVector3DReadOnly effectiveICPAdjustment)
   {
      this.effectiveICPAdjustment.setIncludingFrame(effectiveICPAdjustment);
   }

   public FrameVector3DReadOnly getEffectiveICPAdjustment()
   {
      return effectiveICPAdjustment;
   }
}
