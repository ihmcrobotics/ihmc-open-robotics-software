package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

/**
 * Command that holds output from the {@link LinearMomentumRateControlModule} going to the walking controller state
 * machine that might be running at a slower rate then the ICP feedback.
 *
 * @author Georg Wiedebach
 */
public class LinearMomentumRateControlModuleOutput
{
   /**
    * The desired feedback CMP. It is used by the walking controller to check some conditions (e.g. it's placement
    * before toe off).
    */
   private final FramePoint2D desiredCMP = new FramePoint2D();

   /**
    * If the footstep was adjusted (see {@link #footstepWasAdjusted}) this will contain the shift vector used by the
    * walking controller to possibly update upcoming footsteps and the CoM plan.
    */
   private final FrameVector3D effectiveICPAdjustment = new FrameVector3D();

   /**
    * Flag that indicated that the ICP controller might adjust the upcoming footstep. This does not mean that the step
    * was adjusted (for that see {@link #footstepWasAdjusted}) but merely that the ICP controller assumes it can adjust
    * the step.
    */
   private boolean usingStepAdjustment;

   /**
    * Flag indicating that the upcoming footstep was adjusted. This can happen if there is significant tracking error.
    * If this flag is true {@link #footstepSolution} will contain the pose of the adjusted footstep.
    */
   private boolean footstepWasAdjusted;

   /**
    * If {@link #footstepWasAdjusted} is {@code true} this will contain the new adjusted footstep pose.
    */
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
