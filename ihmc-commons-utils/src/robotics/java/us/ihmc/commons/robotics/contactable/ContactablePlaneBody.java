package us.ihmc.commons.robotics.contactable;

import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;

public interface ContactablePlaneBody extends ContactableBody
{
   ReferenceFrame getContactFrame();

   List<? extends FramePoint2DReadOnly> getContactPoints2D();
}
