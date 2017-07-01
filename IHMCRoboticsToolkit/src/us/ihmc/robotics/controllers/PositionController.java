package us.ihmc.robotics.controllers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;

/**
 * @author twan
 *         Date: 6/15/13
 */
public interface PositionController
{
   public abstract void compute(FrameVector output, FramePoint desiredPosition, FrameVector desiredVelocity, FrameVector currentVelocity,
                FrameVector feedForward);

   public abstract ReferenceFrame getBodyFrame();
}
