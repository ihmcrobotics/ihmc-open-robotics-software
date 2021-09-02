package us.ihmc.valkyrie;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.log.LogTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.Fiducial;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.FloatingFiducialBoxRobot;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationConstructionSetTools.util.ground.RotatableConvexPolygonTerrainObject;
import us.ihmc.simulationConstructionSetTools.util.ground.TrussWithSimpleCollisions;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CylinderTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableCinderBlockTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

@SuppressWarnings("unused")
public class ValkyrieWallEnvironment implements CommonAvatarEnvironmentInterface {
	private final CombinedTerrainObject3D combinedTerrainObject3D;
	private final ArrayList<Robot> environmentRobots = new ArrayList<>();

	private final Random random = new Random(1989L);

	private static final boolean VISUALIZE_BOUNDING_BOXES = false;

	private static final boolean SHOW_FULL_TESTBED = false;

	private static final boolean ADD_CEILING = true;

	public ValkyrieWallEnvironment(String configFile) {
		this(true, configFile);
	}

	public ValkyrieWallEnvironment(boolean setUpGround, String configFile) {
		combinedTerrainObject3D = new CombinedTerrainObject3D("ValkyrieWallEnvironment");

		List<SimWall> walls = new ArrayList<SimWall>();
		
		if (configFile != null) {
			// Read in the world config from the config file, mapped into a SimWorld object
			LogTools.info("Setting up world from " + configFile);
			ClassLoader classLoader = Thread.currentThread().getContextClassLoader();
			File file = new File(configFile);
			ObjectMapper om = new ObjectMapper(new YAMLFactory());
			SimWorld world = new SimWorld();
			try {
				world = om.readValue(file, SimWorld.class);
			} catch (IOException e) {
				e.printStackTrace();
				System.out.println("Unable to load world from " + configFile);
				System.exit(2);
			}
	
			walls = world.walls;
			combinedTerrainObject3D.addTerrainObject(setUpWalls("Walls", walls));
		} else {
			LogTools.warn("No config file specified -- world will be empty");
		}

		if (setUpGround) {
			combinedTerrainObject3D.addTerrainObject(setUpGround("Ground"));
		}

	}

	private CombinedTerrainObject3D setUpWalls(String name, List<SimWall> walls) {
		CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);
		AppearanceDefinition color = YoAppearance.Gray();

		for (SimWall wall : walls) {
			// Walls
			setUpSlopedBox(combinedTerrainObject, wall, color);
		}

//      for (int i = 0; i < 1; i++)
//      {
//         Graphics3DObject linkGraphics = new Graphics3DObject();
//
//         // Vector3d translation = new Vector3d(-1.0, 0, startDistance);// startDistance);
//         Vector3D translation = new Vector3D(-1, 0, 2.9); // startDistance);
//
//         linkGraphics.rotate(Math.PI / 2, Axis3D.Y);
//         linkGraphics.rotate(Math.toRadians(-courseAngleDeg), Axis3D.X);
//         linkGraphics.translate(translation);
//
//         combinedTerrainObject.addStaticLinkGraphics(linkGraphics); // new
//      }

		return combinedTerrainObject;
	}

	private static double[] rotateAroundOrigin(double[] xy, double angdeg) {
		double x = xy[0];
		double y = xy[1];
		double[] newPoint = new double[2];
		double angRad = Math.toRadians(angdeg);
		newPoint[0] = x * Math.cos(angRad) - y * Math.sin(angRad);
		newPoint[1] = y * Math.cos(angRad) + x * Math.sin(angRad);

		return newPoint;
	}

	private static void setUpSlopedBox(CombinedTerrainObject3D combinedTerrainObject, SimWall wall,
			AppearanceDefinition app) {
		RigidBodyTransform location = new RigidBodyTransform();
		location.setRotationYawAndZeroTranslation(Math.toRadians(wall.yawDegrees));

		RigidBodyTransform tilt = new RigidBodyTransform();
		tilt.setRotationPitchAndZeroTranslation(-wall.slopeRadians);
		location.multiply(tilt);

		location.getTranslation().set(wall.center);
		RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(
				new Box3D(location, new Vector3D(wall.lengths)), app);
		combinedTerrainObject.addTerrainObject(newBox);
	}

	public static CombinedTerrainObject3D setUpGround(String name) {
		CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

		// URL fileURL =
		// DRCDemo01NavigationEnvironment.class.getClassLoader().getResource("Textures/ground2.png");
		YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");

		RigidBodyTransform location = new RigidBodyTransform();
		location.getTranslation().set(new Vector3D(0, 0, -0.5));

		RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, 45, 45, 1), texture);
		combinedTerrainObject.addTerrainObject(newBox);
		RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3D(location, 200, 200, 0.75),
				YoAppearance.DarkGray());
		combinedTerrainObject.addTerrainObject(newBox2);

		return combinedTerrainObject;
	}

	@Override
	public TerrainObject3D getTerrainObject3D() {
		return combinedTerrainObject3D;
	}

	@Override
	public ArrayList<Robot> getEnvironmentRobots() {
		return environmentRobots;
	}

	@Override
	public void createAndSetContactControllerToARobot() {

	}

	@Override
	public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints) {

	}

	@Override
	public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener) {

	}

}
