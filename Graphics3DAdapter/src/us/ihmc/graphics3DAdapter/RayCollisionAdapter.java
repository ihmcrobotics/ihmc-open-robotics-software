package us.ihmc.graphics3DAdapter;

import us.ihmc.utilities.math.geometry.Ray3d;

public interface RayCollisionAdapter
{
	public void setPickingGeometry(Ray3d ray3d);
	public double getPickDistance();
}