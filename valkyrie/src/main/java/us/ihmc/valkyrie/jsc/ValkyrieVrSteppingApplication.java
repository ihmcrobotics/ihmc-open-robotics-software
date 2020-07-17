package us.ihmc.valkyrie.jsc;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickBasedSteppingMainUI;
import us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXController.SecondaryControlOption;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Parameter;
import com.martiansoftware.jsap.SimpleJSAP;
import com.martiansoftware.jsap.StringParser;
import com.martiansoftware.jsap.JSAP;

public class ValkyrieVrSteppingApplication {
	private ValkyrieVrSteppingController controller;
	private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS,
			"ihmc_valkyrie_vr_joystick_control");

	public void start(JSAPResult jsapResult) {
		String robotTargetString = jsapResult.getString("robotTarget");
		RobotTarget robotTarget = RobotTarget.valueOf(robotTargetString);
		LogTools.info("-------------------------------------------------------------------");
		LogTools.info("  -------- Loading parameters for RobotTarget: " + robotTarget + "  -------");
		LogTools.info("-------------------------------------------------------------------");
		ValkyrieRobotModel robotModel = new ValkyrieRobotModel(robotTarget, ValkyrieRosControlController.VERSION);
		String robotName = robotModel.getSimpleRobotName();

		WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
		SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
		double footLength = steppingParameters.getFootLength();
		double footWidth = steppingParameters.getFootWidth();
		ConvexPolygon2D footPolygon = new ConvexPolygon2D();
		footPolygon.addVertex(footLength / 2.0, footWidth / 2.0);
		footPolygon.addVertex(footLength / 2.0, -footWidth / 2.0);
		footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
		footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
		footPolygon.update();

		SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(footPolygon, footPolygon);

		controller = new ValkyrieVrSteppingController(robotModel, robotName, walkingControllerParameters, ros2Node, footPolygons);
	}

	public void stop() {
		ros2Node.destroy();
		System.exit(0);
	}

	/**
	 * Argument options:
	 * <ul>
	 * <li>Selecting the robot target: for sim: {@code --robotTarget=SCS}, for
	 * hardware: {@code --robotTarget=REAL_ROBOT}.
	 * <li>Selecting the working directory (where the profiles are saved):
	 * {@code --workingDir=~/home/myWorkingDirectory}. If none provided the default
	 * is set to {@code "~/.ihmc/joystick_step_app/"}.
	 * </ul>
	 * 
	 * @param args the array of arguments to use for this run.
	 */
    public static void main(String[] args) {
		// TODO: Add support for profiles/working directory
		JSAPResult jsapResult = null;
		try {
			final SimpleJSAP jsap = new SimpleJSAP("ValkyrieVrSteppingApplication", "Make the robot walk",
					new Parameter[] 
						{ new FlaggedOption("robotTarget", JSAP.STRING_PARSER, "REAL_ROBOT", JSAP.NOT_REQUIRED, 'r', "robotTarget", "Robot target (REAL_ROBOT, SCS)") });
			jsapResult = jsap.parse(args);
		} catch (JSAPException e) {
			System.out.println("Invalid option: " + e);
			System.exit(0);
		}
		ValkyrieVrSteppingApplication app = new ValkyrieVrSteppingApplication();
		app.start(jsapResult);
//		app.stop();
		
	}
}
