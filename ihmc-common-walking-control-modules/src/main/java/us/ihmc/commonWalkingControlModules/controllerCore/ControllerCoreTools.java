package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public final class ControllerCoreTools
{
   private ControllerCoreTools()
   {
   }

   public static void checkExpressedInRootFrame(ReferenceFrameHolder referenceFrameHolder)
   {
      if (!referenceFrameHolder.getReferenceFrame().isRootFrame())
         throw new IllegalArgumentException("The input has to be expressed in a root frame.");
   }

   public static void checkExpressedInSameRootFrame(ReferenceFrameHolder referenceFrameHolder1, ReferenceFrameHolder referenceFrameHolder2)
   {
      referenceFrameHolder1.checkReferenceFrameMatch(referenceFrameHolder2);
      checkExpressedInRootFrame(referenceFrameHolder1);
   }

   public static void checkExpressedInSameRootFrame(ReferenceFrameHolder referenceFrameHolder1, ReferenceFrameHolder referenceFrameHolder2,
                                                    ReferenceFrameHolder referenceFrameHolder3)
   {
      referenceFrameHolder1.checkReferenceFrameMatch(referenceFrameHolder2);
      referenceFrameHolder1.checkReferenceFrameMatch(referenceFrameHolder3);
      checkExpressedInRootFrame(referenceFrameHolder1);
   }
}
