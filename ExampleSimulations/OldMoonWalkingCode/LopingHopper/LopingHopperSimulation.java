package us.ihmc.moonwalking.models.LopingHopper;

import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class LopingHopperSimulation
{


	public LopingHopperSimulation()
	{
		double gravity = 9.81;// / 6.0;

		LopingHopperRobot lopingHopperRobot = new LopingHopperRobot(gravity);
		LopingSpeedController lopingController = new LopingSpeedController(lopingHopperRobot, gravity);
		lopingHopperRobot.setController(lopingController);
		LinearGroundContactModel linearGroundContactModel = new LinearGroundContactModel(lopingHopperRobot, 15000.0, 2000.0, 1000.0, 3000.0);
		lopingHopperRobot.setGroundContactModel(linearGroundContactModel);

		startSimulationGUI(lopingHopperRobot);
	}

	private void startSimulationGUI(LopingHopperRobot oneLeggedHopperRobot)
	{

		SimulationConstructionSet scs = new SimulationConstructionSet(oneLeggedHopperRobot);

		setUpGUI(scs);
		// scs.selectConfiguration("Control");
		scs.selectConfiguration("Control");

		// scs.setGroundVisible(false);

		scs.setDT(0.0001, 200);

		Thread thread = new Thread(scs);
		thread.setName("SimulationConstructionSet");
		thread.start();

	}

	private void setUpGUI(SimulationConstructionSet scs)
	{
		CameraConfiguration view1 = new CameraConfiguration("view1");
		view1.setCameraDolly(false, true, true, false);
		view1.setCameraTracking(false, true, true, false);
		view1.setCameraPosition(0.0, -9.6, 1.2);
		view1.setCameraFix(-0.29, 0.065, 0.88);
		scs.setupCamera(view1);

		scs.selectCamera("view1");

		scs.setupGraphGroup("Control", new String[][][] { { { "qd_x" }, { "auto" } }, { { "q_frontLegJoint", "q_d_frontLegJoint" }, { "auto" } },

		{ { "q_backLegJoint", "q_d_backLegJoint" }, { "auto" } },

		{ { "front_state", "back_state" }, { "auto" } },

		}, 1);

		scs.setupEntryBoxGroup("Control", new String[] { "frontLegLengthExtend", "frontLegLengthRetract", "backLegLengthExtend", "backLegLengthRetract", "frontHipForword", "frontHipBackword", "backHipForword", "backHipBackword" });

		scs.setupConfiguration("Control", "all", "Control", "Control");
	}

	public static void main(String[] args)
	{
		LopingHopperSimulation oneLeggedHopperSimulation = new LopingHopperSimulation();
	}
}
