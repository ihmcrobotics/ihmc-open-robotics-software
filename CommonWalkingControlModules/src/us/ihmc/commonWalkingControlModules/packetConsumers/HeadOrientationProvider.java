package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface HeadOrientationProvider
{
   public abstract boolean isNewLookAtInformationAvailable();

   public abstract boolean isNewHeadOrientationInformationAvailable();

   public abstract FrameOrientation getDesiredHeadOrientation();

   public abstract FramePoint getLookAtPoint();

   public abstract ReferenceFrame getHeadOrientationExpressedInFrame();
}