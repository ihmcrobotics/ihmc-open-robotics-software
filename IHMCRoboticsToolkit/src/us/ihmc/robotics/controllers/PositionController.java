package us.ihmc.robotics.controllers;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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
