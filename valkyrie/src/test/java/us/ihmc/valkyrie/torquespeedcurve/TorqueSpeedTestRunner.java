package us.ihmc.valkyrie.torquespeedcurve;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.core.Version;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.module.SimpleModule;
import org.yaml.snakeyaml.Yaml;
import com.google.common.io.Files;
import com.google.gson.Gson;
import com.google.gson.annotations.Expose;
import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataListMessagePubSubType;
import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessagePubSubType;
import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutatorList;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSDFDescriptionMutator;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieJointTorqueLimitMutator;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieTorqueSpeedCurveEndToEndTestNasa;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieTorqueSpeedTestConfig;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieSDFDescriptionMutator;

import us.ihmc.idl.serializers.extra.JSONSerializer;

public class TorqueSpeedTestRunner {
	private double inchesToMeters(double inches) {
		return inches * 2.54 / 100.0;
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
	
	public void disableAnkleLimits(ValkyrieRobotModel robotModel)
	{
		robotModel.setSDFDescriptionMutator(new ValkyrieSDFDescriptionMutator(robotModel.getJointMap(), true)
		{
			@Override
			public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
			{
				if (jointHolder.getName().contains("AnklePitch"))
					jointHolder.setLimits(-Math.PI, Math.PI);
			}
		});
	}	
	
	private ValkyrieSDFDescriptionMutator getAnkleLimitMutator(ValkyrieRobotModel robot)
	{ 
		return new ValkyrieSDFDescriptionMutator(robot.getJointMap(), true) {
			@Override
			public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
			{
				if (jointHolder.getName().contains("AnklePitch"))
					jointHolder.setLimits(-Math.PI, Math.PI);
			}
		};
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
		ValkyrieTorqueSpeedTestConfig config = null;

		try (Reader reader = new FileReader(paramFile)) {
			config = gson.fromJson(reader, ValkyrieTorqueSpeedTestConfig.class);
		} catch (IOException e1) {
			System.err.printf("Error reading test params from %s: %s\n", paramFile, e1.toString());
			System.exit(1);
		}
		System.out.println(config.toString());

		// If the config file specifies footsteps, read them in here. Otherwise
		// recordedFootsteps will be null.
		FootstepDataListMessage recordedFootsteps = config.getFootsteps();
		
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

		// Create test runner and test case class
		TorqueSpeedTestRunner runner = new TorqueSpeedTestRunner();
		ValkyrieTorqueSpeedCurveEndToEndTestNasa tester = new ValkyrieTorqueSpeedCurveEndToEndTestNasa();

		// Disable ankle limits on the robot
		if (config.disableAnkleLimits) {
			mutator.addMutator(runner.getAnkleLimitMutator(robot));
		}		

		robot.setSDFDescriptionMutator(mutator);
		
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
			case STEP:
				outputResultsDirectory = tester.testStepUpWithoutSquareUp(robot, config.stepStartingDistance, 
						config.stepHeight, walkingParameters, recordedFootsteps, outputPrefixDirectory);
				break;			
			case SLOPE:
				outputResultsDirectory = tester.testWalkSlope(robot, 
						                                      Math.toRadians(-config.slopeDegrees), 
						                                      runner.inchesToMeters(config.stepLengthInches),
						                                      walkingParameters, 
						                                      recordedFootsteps, 
						                                      outputPrefixDirectory);
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
		
		// Without calling System.exit(0), the sim has non-terminating threads. Specifically, the simulation
		// thread and the intraprocess thread (associated with ROS2 publishing) do not exit.
		System.exit(0);
	}

}
