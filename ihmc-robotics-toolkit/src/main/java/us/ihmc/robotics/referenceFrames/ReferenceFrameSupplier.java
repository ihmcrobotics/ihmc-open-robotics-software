package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import java.util.function.Supplier;

/**
 * Just a tool to make this one symbol since it's gotta get passed around a lot.
 * Sorta like a typedef of a pointer pointer.
 */
public interface ReferenceFrameSupplier extends Supplier<ReferenceFrame>
{

}
