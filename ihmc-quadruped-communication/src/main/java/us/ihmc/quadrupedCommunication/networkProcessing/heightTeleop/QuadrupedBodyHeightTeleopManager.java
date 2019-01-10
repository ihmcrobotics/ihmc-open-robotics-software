package us.ihmc.quadrupedCommunication.networkProcessing.heightTeleop;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.*;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.bodyPath.QuadrupedBodyPathMultiplexer;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapper;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitStepStream;

import java.util.concurrent.atomic.AtomicBoolean;

public class QuadrupedBodyHeightTeleopManager
{
   private final AtomicBoolean paused = new AtomicBoolean(false);
   private final AtomicDouble desiredBodyHeight = new AtomicDouble();

   private final QuadrupedReferenceFrames referenceFrames;

   private QuadrupedBodyHeightMessage bodyHeightMessage;

   public QuadrupedBodyHeightTeleopManager(double initialBodyHeight, QuadrupedReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;

      desiredBodyHeight.set(initialBodyHeight);
   }

   public void initialize()
   {
      populateBodyHeightMessage();
   }

   public void update()
   {
      bodyHeightMessage = null;

      if (paused.get())
         return;

      populateBodyHeightMessage();
   }

   public void sleep()
   {
      paused.set(true);
   }

   public QuadrupedBodyHeightMessage getBodyHeightMessage()
   {
      return bodyHeightMessage;
   }

   public void setDesiredBodyHeight(double desiredBodyHeight)
   {
      this.desiredBodyHeight.set(desiredBodyHeight);
   }

   private final FramePoint3D tempPoint = new FramePoint3D();

   private void populateBodyHeightMessage()
   {
      double bodyHeight = desiredBodyHeight.getAndSet(Double.NaN);

      if (!Double.isNaN(bodyHeight))
      {
         tempPoint.setIncludingFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds(), 0.0, 0.0, bodyHeight);
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());

         bodyHeightMessage = QuadrupedMessageTools.createQuadrupedBodyHeightMessage(0.0, tempPoint.getZ());
         bodyHeightMessage.setControlBodyHeight(true);
         bodyHeightMessage.setIsExpressedInAbsoluteTime(false);
      }
   }

   public void setPaused(boolean pause)
   {
      paused.set(pause);
   }

   public boolean isPaused()
   {
      return paused.get();
   }
}