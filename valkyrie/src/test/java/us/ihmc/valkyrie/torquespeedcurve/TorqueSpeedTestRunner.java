package us.ihmc.valkyrie.torquespeedcurve;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.HashSet;

import com.google.common.io.Files;
import com.google.gson.Gson;
import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutatorList;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSDFDescriptionMutator;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieJointTorqueLimitMutator;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieTorqueSpeedCurveEndToEndTestNasa;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class TorqueSpeedTestRunner {

	// Nested class containing configuration parameters for this test
	// For reasons I don't understand, if TestConfig is not marked "static", GSON
	// will set any
	// fields not specified in the parameter file to null/0.
	static class TestConfig {
		enum TestType {
			STAIRS, SQUARE_UP_STEP;
		}

		public double stepStartingDistance;
		public double stepHeight;
		public int numberOfSteps;
		public HashMap<String, Double> torqueLimits;
		public boolean showGui;
		public TestType testType;

		// Default constructor
		public TestConfig() {
			stepStartingDistance = 1.0 * 100.0 / 2.54; // 1m in inches
			stepHeight = 6.0;
			numberOfSteps = 3;
			torqueLimits = new HashMap<String, Double>();
			showGui = true;
			testType = TestType.STAIRS;
		}

		public String toString() {
			String value = String.format("Test Type: %s\nStep Starting Distance: %f\nStep Height: %f\nNumber of Steps: %d\nShow Gui: %b\n",
					testType, stepStartingDistance, stepHeight, numberOfSteps, showGui);
			for (String joint : torqueLimits.keySet()) {
				value += String.format("%s joint torque limit: %f\n", joint, torqueLimits.get(joint).doubleValue());
			}
			return value;
		}
	}

	// Given a folder name, construct a top-level output directory
	private File getOutputDirectory(String folderName) {
		String homeDirectory = System.getProperty("user.home");
		Path path = Paths.get(homeDirectory, "test_results", folderName);
		try {
			FileTools.ensureDirectoryExists(path);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return path.toFile();
	}

	private static JSAP getArgumentParser() {
		JSAP argumentParser = new JSAP();
		FlaggedOption paramFileOption = new FlaggedOption("paramfile").setShortFlag('f').setLongFlag("paramfile")
				.setRequired(true);

		try {
			argumentParser.registerParameter(paramFileOption);
		} catch (JSAPException e1) {
			System.err.println("Unable to register option parameters");
			e1.printStackTrace();
			System.exit(1);
		}

		return argumentParser;
	}

	public static void main(String[] args) {

		JSAP argumentParser = getArgumentParser();
		JSAPResult arguments = argumentParser.parse(args);

		String paramFile = arguments.getString("paramfile");
		if (paramFile == null) {
			System.err.println("A parameter file must be specified.");
			System.err.println(argumentParser.getUsage());
			System.exit(1);
		}
		System.out.printf("Got param file of %s\n", paramFile);

		File paramInput = new File(paramFile);
		if (!paramInput.exists()) {
			System.err.printf("Parameter file %s does not exist\n", paramFile);
			System.exit(1);
		}

		// Read in the test configuration from the test parameters file
		Gson gson = new Gson();
		TestConfig config = null;

		try (Reader reader = new FileReader(paramFile)) {
			config = gson.fromJson(reader, TestConfig.class);
		} catch (IOException e1) {
			System.err.printf("Error reading test params from %s: %s\n", paramFile, e1.toString());
			System.exit(1);
		}
		System.out.println(config.toString());

		ValkyrieRobotModel robot = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT);

		// Set walking parameters
		WalkingControllerParameters walkingParameters = robot.getWalkingControllerParameters();

		// Create a list of mutators based on the torque limits defined in the params
		// file
		SDFDescriptionMutatorList mutator = new SDFDescriptionMutatorList(
				new ValkyrieSDFDescriptionMutator(robot.getJointMap(), false));

		// Create a list of joint names with the format we need
		HashSet<String> validJointNames = new HashSet<String>();
		for (LegJointName joint : LegJointName.values()) {
			if (!joint.equals("Unknown Position")) {
				validJointNames.add(joint.getPascalCaseName());
			}
		}

		// Add a mutator for each joint torque limit specified in the parameter file
		for (String joint : config.torqueLimits.keySet()) {
			if (!validJointNames.contains(joint)) {
				System.err.println("Invalid joint specified in config file: " + joint);
				System.err.println("Ensure case and spelling are correct (e.g. KneePitch)");
				System.exit(1);
			}
			mutator.addMutator(
					new ValkyrieJointTorqueLimitMutator(robot.getJointMap(), joint, config.torqueLimits.get(joint)));
		}
		robot.setSDFDescriptionMutator(mutator);

		// Create test runner and test case class
		TorqueSpeedTestRunner runner = new TorqueSpeedTestRunner();
		ValkyrieTorqueSpeedCurveEndToEndTestNasa tester = new ValkyrieTorqueSpeedCurveEndToEndTestNasa();

		// Set whether tester should show a GUI
		tester.setGuiEnabled(config.showGui);

		// Set up output directories for test results
		File outputPrefixDirectory = runner.getOutputDirectory("capped_torque");
		File outputResultsDirectory = null;

		// Run the test
		try {
			switch (config.testType) {
			case STAIRS:
				outputResultsDirectory = tester.testUpstairs(robot, config.stepHeight, config.numberOfSteps,
						walkingParameters, outputPrefixDirectory);
				break;
			case SQUARE_UP_STEP:
				outputResultsDirectory = tester.testStepUpWithSquareUp(robot, config.stepStartingDistance, 
						config.stepHeight, walkingParameters, outputPrefixDirectory);
				break;
			}

		} catch (SimulationExceededMaximumTimeException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Copy the test parameters into the results directory
		try {
			if (outputResultsDirectory != null) {
				Files.copy(paramInput, Paths
						.get(outputResultsDirectory.toString(), paramInput.toPath().getFileName().toString()).toFile());
			}
		} catch (IOException e) {
			System.err.println("Unable to copy param file to destination: " + e.getMessage());
		}
	}
}
