package us.ihmc.graphics3DAdapter;

import us.ihmc.utilities.math.geometry.Ray;

public interface PickingAdapter
{
	public void setPickingGeometry(Ray ray);
	public double getPickDistance();
}