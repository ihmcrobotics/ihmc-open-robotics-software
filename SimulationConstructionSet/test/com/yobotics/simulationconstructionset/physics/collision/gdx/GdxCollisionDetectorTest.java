package com.yobotics.simulationconstructionset.physics.collision.gdx;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.physics.ScsCollisionDetector;
import com.yobotics.simulationconstructionset.physics.collision.GenericCollisionChecks;

/**
 * @author Peter Abeles
 */
public class GdxCollisionDetectorTest extends GenericCollisionChecks
{
	@Override
	public ScsCollisionDetector createCollisionInterface()
	{
		return new GdxCollisionDetector(new YoVariableRegistry("Dummy"),1000);
	}
}
