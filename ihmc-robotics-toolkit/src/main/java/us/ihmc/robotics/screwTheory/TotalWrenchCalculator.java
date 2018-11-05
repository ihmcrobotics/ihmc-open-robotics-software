package us.ihmc.robotics.screwTheory;

import java.util.Collection;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public class TotalWrenchCalculator
{
   private final Wrench temporaryWrench = new Wrench();

   public void computeTotalWrench(Wrench totalGroundReactionWrenchToPack, Collection<? extends WrenchReadOnly> wrenches, ReferenceFrame referenceFrame)
   {
      totalGroundReactionWrenchToPack.setToZero(referenceFrame, referenceFrame);

      for (WrenchReadOnly wrench : wrenches)
      {
         temporaryWrench.setIncludingFrame(wrench);
         temporaryWrench.changeFrame(referenceFrame);
         temporaryWrench.setBodyFrame(referenceFrame);
         totalGroundReactionWrenchToPack.add(temporaryWrench);
      }
   }
}
