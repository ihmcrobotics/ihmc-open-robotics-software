package us.ihmc.graphics3DAdapter;

import us.ihmc.utilities.math.geometry.Ray;

public interface RayCollisionAdapter
{
	public void setPickingGeometry(Ray ray);
	public double getPickDistance();
}