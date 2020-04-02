package us.ihmc.valkyrie.testsupport;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Specifies a configuration interface for modifying various aspects of Valkyrie
 * @author mark
 *
 */
public interface ModifiableValkyrieRobotConfig {
	public HashMap<String, Double> getTorqueLimits();
	public HashMap<String, Double> getVelocityLimits();
	public HashMap<String, Double> getLinkMassKg();
	public HashMap<String, ArrayList<Double>> getPositionLimits();
	public double getGlobalMassScale();
	public double getGlobalSizeScale();
	public boolean getAnkleLimitsDisabled();
	public boolean getMinimizeJointTorques();
}
