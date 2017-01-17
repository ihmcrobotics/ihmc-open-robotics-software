package us.ihmc.simulationconstructionset;

import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface SimulatedSensor {
	public void updateTransform(RigidBodyTransform transformToHere, double time);
	public void setWorld(Graphics3DAdapter graphics3dAdapter);
	public String getName();
	public RigidBodyTransform getTransformToHere();
}
