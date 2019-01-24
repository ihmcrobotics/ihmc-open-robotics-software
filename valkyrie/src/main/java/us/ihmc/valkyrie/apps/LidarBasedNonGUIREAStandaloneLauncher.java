package us.ihmc.valkyrie.apps;

import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;

public class LidarBasedNonGUIREAStandaloneLauncher {
	private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

	public static void main(String[] args) {
		LIDARBasedREAModule module;

		try {
			module = LIDARBasedREAModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME);
			module.start();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
