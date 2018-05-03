package us.ihmc.jMonkeyEngineToolkit;

import us.ihmc.euclid.geometry.Line3D;

public interface RayCollisionAdapter
{
	public void setPickingGeometry(Line3D ray3d);
	public double getPickDistance();
}