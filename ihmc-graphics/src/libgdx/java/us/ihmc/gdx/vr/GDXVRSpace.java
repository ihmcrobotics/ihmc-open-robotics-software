package us.ihmc.gdx.vr;

import com.badlogic.gdx.math.Matrix4;

/**
 * Space in which matrices and vectors are returned in by
 * {@link GDXVRDevice} methods taking a {@link GDXVRSpace}.
 * In case {@link GDXVRSpace#World} is specified, all values
 * are transformed by the {@link Matrix4} set via
 * {@link GDXVRContext#setTrackerSpaceOriginToWorldSpaceTransform(Matrix4)}.
 */
public enum GDXVRSpace
{
   Tracker, World
}
