package us.ihmc.simulationconstructionset;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;

public interface SimulatedSensor {
	public void updateTransform(RigidBodyTransform transformToHere, double time);
	public void setWorld(Graphics3DAdapter graphics3dAdapter);
	public String getName();
}
