package us.ihmc.valkyrie;

import java.io.IOException;
import java.net.URI;
import java.util.Collection;
import java.util.Collections;

import org.ros.internal.message.Message;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.avatar.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.avatar.rosAPI.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;
import us.ihmc.valkyrie.parameters.ValkyrieContactPointParameters;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class OpenHumanoidsSimulator
{
   private static final String ROBOT_NAME = "valkyrie";
   private static final String DEFAULT_PREFIX = "/ihmc_ros";
   private static final boolean START_UI = false;
   private static final boolean REDIRECT_UI_PACKETS_TO_ROS = false;
   private static final String DEFAULT_TF_PREFIX = null;
   private SDFEnvironment environment = null;
   private AvatarSimulation avatarSimulation;
   private boolean rosShutDown = false;

   class RosIPABAPISubscriber extends AbstractRosTopicSubscriber<std_msgs.String>
   {

	   	public RosIPABAPISubscriber()
	   	{
	   		super(std_msgs.String._TYPE);
	   	}


	   	@Override
	   	public void onNewMessage(std_msgs.String message)
	   	{
	   		callbackROSAPI(message.getData());
	   	}
   }

	public OpenHumanoidsSimulator(String model, DRCStartingLocation startingLocation, String nameSpace, String tfPrefix,
			boolean runAutomaticDiagnosticRoutine, boolean disableViz, boolean extra_sim_points) throws IOException
	{
		this(model, startingLocation, nameSpace, tfPrefix, runAutomaticDiagnosticRoutine, disableViz, extra_sim_points, Collections.<Class>emptySet());
	}

   public OpenHumanoidsSimulator(String model, DRCStartingLocation startingLocation, String nameSpace, String tfPrefix,
         boolean runAutomaticDiagnosticRoutine, boolean disableViz, boolean extra_sim_points, Collection<Class> additionalPacketTypes) throws IOException
   {
         FootContactPoints simulationContactPoints = null;
         if (extra_sim_points)
         {
            simulationContactPoints = new AdditionalSimulationContactPoints(8, 3, false, true);
            System.out.println("Added extra foot contact points.");
         }
	      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false, model, simulationContactPoints);

	      //TODO: Get this stuff from the RobotDescription rather than the SDF stuff...
	      GeneralizedSDFRobotModel generalizedSDFRobotModel = robotModel.getGeneralizedRobotModel();

	      if (load_sdf_contacts)
	      {
	         ValkyrieContactPointParameters contactPointParameters = (ValkyrieContactPointParameters) robotModel.getContactPointParameters();
	         contactPointParameters.setupContactPointsFromRobotModel(generalizedSDFRobotModel, replace_contacts);
	      }

	      environment = new SDFEnvironment();

	      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
	      simulationStarter.setRunMultiThreaded(true);

		  DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();

		  URI rosUri = NetworkParameters.getROSURI();
		  networkProcessorParameters.setRosUri(rosUri);

		  PacketCommunicator rosAPI_communicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, new us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList());

		  networkProcessorParameters.enableROSAPICommunicator(true);
		  if (runAutomaticDiagnosticRoutine)
		  {
		     networkProcessorParameters.enableBehaviorModule(true);
		     networkProcessorParameters.enableBehaviorVisualizer(true);
		     networkProcessorParameters.enableAutomaticDiagnostic(true, 5);
		  }

		  if (START_UI)
		  {
		     networkProcessorParameters.enableUiModule(true);
		     System.err.println("Cannot start UI automatically from open source projects. Start UI Manually");
		  }

		  if(disableViz)
		  {
		     DRCGuiInitialSetup guiSetup = new DRCGuiInitialSetup(false, false, false);
		     simulationStarter.setGuiInitialSetup(guiSetup);
		  }

		  simulationStarter.setStartingLocation(startingLocation);
		  simulationStarter.setInitializeEstimatorToActual(true);
		  simulationStarter.startSimulation(networkProcessorParameters, false);
		  simulationStarter.getAvatarSimulation().getSimulationConstructionSet().hideAllDynamicGraphicObjects();


		  if (REDIRECT_UI_PACKETS_TO_ROS)
		  {
		     PacketRouter<PacketDestination> packetRouter = simulationStarter.getPacketRouter();
		     new UiPacketToRosMsgRedirector(robotModel, rosUri, rosAPI_communicator, packetRouter, DEFAULT_PREFIX + "/" + robotModel.getSimpleRobotName().toLowerCase());
		  }

		  LocalObjectCommunicator sensorCommunicator = simulationStarter.getSimulatedSensorsPacketCommunicator();
		  SimulationRosClockPPSTimestampOffsetProvider ppsOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();

		  java.util.List<java.util.Map.Entry<String,RosTopicSubscriberInterface<? extends Message>>> subscribers = new java.util.ArrayList<>();

		  RosTopicSubscriberInterface<? extends Message> sub=new RosIPABAPISubscriber();
		  java.util.Map.Entry<String,RosTopicSubscriberInterface<? extends Message>> pair=new java.util.AbstractMap.SimpleEntry<String,RosTopicSubscriberInterface<? extends Message>>(nameSpace+"/api_command", sub);
		  subscribers.add(pair);

		  new ThePeoplesGloriousNetworkProcessor(rosUri, rosAPI_communicator, sensorCommunicator, ppsOffsetProvider, robotModel, nameSpace, tfPrefix, additionalPacketTypes, subscribers, null);

		  avatarSimulation = simulationStarter.getAvatarSimulation();
   }

   private void processCommand(String command)
   {
	   if(command.trim().toLowerCase().equals("simulate")) simulate();
	   if(command.trim().toLowerCase().equals("stop"))
	   {
		   System.out.println("Shutting down simulator");
		   System.exit(0);
	   }
	   if(command.trim().toLowerCase().startsWith("loadsdf")) loadEnvironment(command.substring(7).trim());
   }

   public void callbackROSAPI(String data)
   {
	   String[] commands = data.split("\n");
	   if(commands.length==0)
	   {
		   processCommand(data);
	   }
	   else
	   {
		   for(String command : commands)
		   {
			   processCommand(command);
		   }
	   }
   }

   public void simulate()
   {
	   System.out.println("Starting simulation");
	   avatarSimulation.simulate();
   }

   public void loadEnvironment(String filename)
   {
	   System.out.println("Loading environment from '"+filename+"'");
	   environment.load(filename);
	   avatarSimulation.updateEnvironment(environment);
   }

   public static boolean extra_sim_points = false;
   public static String robotModel = "DEFAULT";
   public static boolean load_sdf_contacts = false;
   public static boolean replace_contacts = false;

   public static void parseArguments(String[] args) throws JSAPException
   {
	   JSAP jsap = new JSAP();

	   Switch extra_sim = new Switch("extra-foot-contact-points").setShortFlag('f').setLongFlag("extra-foot-contact-points");
	   extra_sim.setHelp("Adds additional contact points to the simulator (not the conroller)");

	   FlaggedOption model = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(false).setStringParser(JSAP.STRING_PARSER);
	   model.setHelp("Set robot SDF/URDF");
       model.setDefault("DEFAULT");

       Switch sdf_contacts_keep = new Switch("sdf-contact-points").setShortFlag('s').setLongFlag("sdf-contact-points");
       sdf_contacts_keep.setHelp("Creates additional contact points from the SDF file.");
       Switch sdf_contacts_replace = new Switch("sdf-contact-points-replace").setShortFlag('S').setLongFlag("sdf-contact-points-replace");
       sdf_contacts_replace.setHelp("Replaces existing contact points with ones specified in the SDF file.");

	   jsap.registerParameter(extra_sim);
	   jsap.registerParameter(model);
	   jsap.registerParameter(sdf_contacts_keep);
	   jsap.registerParameter(sdf_contacts_replace);
       JSAPResult config = jsap.parse(args);

       robotModel = config.getString(model.getID());
       extra_sim_points = config.getBoolean(extra_sim.getID());
       replace_contacts = config.getBoolean(sdf_contacts_replace.getID());
       load_sdf_contacts = config.getBoolean(sdf_contacts_keep.getID()) || config.getBoolean(sdf_contacts_replace.getID());
   }

   public static void main(String[] args) throws JSAPException, IOException
   {
	   parseArguments(args);

	   OpenHumanoidsSimulator sim = new OpenHumanoidsSimulator(robotModel, DRCObstacleCourseStartingLocation.DEFAULT, DEFAULT_PREFIX + "/" + ROBOT_NAME, DEFAULT_TF_PREFIX, false, false, extra_sim_points);
   }
}


