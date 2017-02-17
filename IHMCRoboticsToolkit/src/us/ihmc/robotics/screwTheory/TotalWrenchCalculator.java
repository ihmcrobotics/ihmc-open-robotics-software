package us.ihmc.robotics.screwTheory;

import java.util.Collection;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class TotalWrenchCalculator
{
   private final Wrench temporaryWrench = new Wrench();

   public void computeTotalWrench(Wrench totalGroundReactionWrenchToPack, Collection<Wrench> wrenches, ReferenceFrame referenceFrame)
   {
      totalGroundReactionWrenchToPack.setToZero(referenceFrame, referenceFrame);

      for (Wrench wrench : wrenches)
      {
         temporaryWrench.set(wrench);
         temporaryWrench.changeFrame(referenceFrame);
         temporaryWrench.changeBodyFrameAttachedToSameBody(referenceFrame);
         totalGroundReactionWrenchToPack.add(temporaryWrench);
      }
   }
}
