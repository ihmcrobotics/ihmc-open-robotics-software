package us.ihmc.exampleSimulations.selfStablePlanarRunner;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SimulationDoneListener;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;


public class SelfStablePlanarRunner_Simulation implements SimulationDoneListener {

    public static final double 		DT		= 0.0001;
    public static final double 		SimulationTime	= 50;
    public static final int 		DataFreq 	= 100;
    public static final int 		BufferSize 	= (int)(SimulationTime/DataFreq/DT+1);
    public static final boolean 	showGUI		= true;
    public static final boolean 	showMarkers 	= true;

    private final double groundKxy = 40000;
    private final double groundBxy =  100;
    private final double groundKz  =   80;
    private final double groundBz  =  500;

    private GroundProfile3D groundProfile  = null;
    private Graphics3DObject  groundGraphics = null;

    private SimulationConstructionSet scs;

    public SelfStablePlanarRunner_Simulation() throws SimulationExceededMaximumTimeException {

	double deltaY = 1.0;
	SelfStablePlanarRunner_Robot[] robots = new SelfStablePlanarRunner_Robot[]{
		new SelfStablePlanarRunner_Robot("SelfStableRunner1", -1.5, -3.0*deltaY),
		new SelfStablePlanarRunner_Robot("SelfStableRunner2", -1.0, -2.0*deltaY),
		new SelfStablePlanarRunner_Robot("SelfStableRunner3", -0.5, -1.0*deltaY),
		new SelfStablePlanarRunner_Robot("SelfStableRunner4",  0.0,  0.0*deltaY),
		new SelfStablePlanarRunner_Robot("SelfStableRunner5",  0.5,  1.0*deltaY),
		new SelfStablePlanarRunner_Robot("SelfStableRunner6",  1.0,  2.0*deltaY),
		new SelfStablePlanarRunner_Robot("SelfStableRunner7",  1.5,  3.0*deltaY),
	};

	System.out.println("Robot mass: " + robots[0].computeCenterOfMass(new Point3D()));

	GroundContactModel[] groundModels = new GroundContactModel[robots.length];
	initGroundProfile();
	for (int i = 0; i < robots.length; i++) {
	    groundModels[i] = new LinearGroundContactModel(robots[i], groundKxy, groundBxy, groundKz, groundBz, robots[i].getRobotsYoVariableRegistry());
	    groundModels[i].setGroundProfile3D(groundProfile);
	    robots[i].setGroundContactModel(groundModels[i]);
	}

	for (int i = 0; i < robots.length; i++) {
	    robots[i].setController(new Controller(robots[i]));
	}


   SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
   parameters.setCreateGUI(showGUI);
   parameters.setDataBufferSize(BufferSize);
   
	scs = new SimulationConstructionSet(robots, parameters);

	// Add a dynamic graphics representation to the push forces
	YoGraphicsListRegistry registries = new YoGraphicsListRegistry();
	YoGraphicsList yoGraphicsList = new YoGraphicsList("DynamicGraphicObjectsList");

	for (SelfStablePlanarRunner_Robot robot: robots) {
	    ArrayList<GroundContactPoint> ef_point_list = robot.getAllGroundContactPoints();
	    for (GroundContactPoint ef_point:ef_point_list) {
		YoGraphicVector pushForceDynGraphVector = new
			YoGraphicVector("PushVector_" + ef_point.getName(), ef_point.getYoPosition(), ef_point.getYoForce(),
				1e-3, YoAppearance.Yellow(), true, 0.08);
		yoGraphicsList.add(pushForceDynGraphVector);
	    }
	}
	registries.registerYoGraphicsList(yoGraphicsList);
	scs.addYoGraphicsListRegistry(registries);

	scs.setDT(DT, DataFreq);
	scs.setFastSimulate(true);
	scs.maximizeMainWindow();
	scs.setCameraFix     ( 4.38, -2.47, 1.05);
	scs.setCameraPosition(13.13, 18.19, 3.56);
//	scs.setCameraDollyOffsets(0, 10, 0);
//	scs.setCameraTracking(true, true, true, false);
//	scs.setCameraDolly(true, true, true, false);

	groundGraphics = new Graphics3DObject();
	scs.addStaticLinkGraphics(groundGraphics);


	Thread myThread = new Thread(scs);
	myThread.start();

	scs.simulate(SimulationTime);
    }

    private void initGroundProfile() {
	double width = 10;
	groundProfile = new FlatGroundProfile(-100, 10000, -width/2.0, width/2.0, 0);

	groundGraphics = new Graphics3DObject();

	// Display markers on the ground to visualize forward speed
	if (showMarkers) {
	    Graphics3DObject lineGraphics = new Graphics3DObject();
	    int numberOfMarkers = 2000, numberOfSubMarkers = 0;
	    double markerSpacingAlongX = 5.0, markerSpacingAlongY = 0.0, xCurrent = 0.0, yCurrent = width/2.0, zCurrent = 0.0, zPrev = 0.0;
	    lineGraphics.translate(xCurrent, yCurrent, 0.0);

	    for (int i = 0; i < numberOfMarkers*(numberOfSubMarkers+1); i++) {
		if (i%(numberOfSubMarkers+1) == 0 && numberOfSubMarkers != 0) {
		    lineGraphics.addGenTruncatedCone(0.5, 0.25, 0.25, 0.1, 0.1, YoAppearance.Red());
		} else {
		    lineGraphics.addGenTruncatedCone(0.5/2, 0.25/2, 0.25/2, 0.1/2, 0.1/2, YoAppearance.AluminumMaterial());
		}
		xCurrent += markerSpacingAlongX/((double)(numberOfSubMarkers+1));
		yCurrent += markerSpacingAlongY;
		zPrev    = zCurrent;
		zCurrent = groundProfile.getHeightMapIfAvailable().heightAt(xCurrent, yCurrent, 0.0);
		lineGraphics.translate(markerSpacingAlongX/((double)(numberOfSubMarkers+1)), markerSpacingAlongY, zCurrent-zPrev);
	    }
	    groundGraphics.combine(lineGraphics);
	}

    }
    
    public void simulationDone() {
    }

    public void simulationDoneWithException(Throwable throwable) {
    }

    public static void main(String[] args) throws SimulationExceededMaximumTimeException
    {
	new SelfStablePlanarRunner_Simulation();
    }
}
