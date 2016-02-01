package us.ihmc.valkyrie;

import java.io.IOException;
import java.net.URI;

import org.ros.internal.message.Message;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JointPositionControllerFactory;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationStarter;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;
import us.ihmc.valkyrie.parameters.ValkyrieContactPointParameters;

public class OpenHumanoidsSimulator
{
   private static final String ROBOT_NAME = "valkyrie";
   private static final String DEFAULT_PREFIX = "/ihmc_ros";
   private static final boolean START_UI = false;
   private static final boolean REDIRECT_UI_PACKETS_TO_ROS = false;
   private static final String DEFAULT_TF_PREFIX = null;
   private SDFEnvironment environment = null;
   private DRCSimulationFactory drcSimulationFactory;
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
	      DRCRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false, model);
	      if(load_sdf_contacts)
	      {
	    	  ValkyrieContactPointParameters contactPointParameters = (ValkyrieContactPointParameters) robotModel.getContactPointParameters();
		      contactPointParameters.setupContactPointsFromRobotModel(robotModel, replace_contacts);
	      }
	      if(extra_sim_points)
	      {
	        ValkyrieContactPointParameters contactPointParameters = (ValkyrieContactPointParameters) robotModel.getContactPointParameters();
	        contactPointParameters.addMoreFootContactPointsSimOnly();
	        System.out.println("Added extra foot contact points.");
	      }
	      
	      environment = new SDFEnvironment();
	      
	   	  DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
		  simulationStarter.setRunMultiThreaded(true);
		  	
		  DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
		  
		  URI rosUri = NetworkParameters.getROSURI();
		  networkProcessorParameters.setRosUri(rosUri);
		
		  PacketCommunicator gfe_communicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.GFE_COMMUNICATOR, new us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList());
		
		  networkProcessorParameters.enableGFECommunicator(true);
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
		
		  simulationStarter.registerHighLevelController(new JointPositionControllerFactory(false));
		  simulationStarter.setStartingLocation(startingLocation);
		  simulationStarter.setInitializeEstimatorToActual(true);
		  simulationStarter.startSimulation(networkProcessorParameters, false);
		  simulationStarter.getDRCSimulationFactory().getSimulationConstructionSet().hideAllDynamicGraphicObjects();
		  
		
		  if (REDIRECT_UI_PACKETS_TO_ROS)
		  {
		     PacketRouter<PacketDestination> packetRouter = simulationStarter.getPacketRouter();
		     new UiPacketToRosMsgRedirector(robotModel, rosUri, gfe_communicator, packetRouter, DEFAULT_PREFIX + "/" + robotModel.getSimpleRobotName().toLowerCase());
		  }
		  
		  LocalObjectCommunicator sensorCommunicator = simulationStarter.getSimulatedSensorsPacketCommunicator();
		  SimulationRosClockPPSTimestampOffsetProvider ppsOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();
		  
		  java.util.List<java.util.Map.Entry<String,RosTopicSubscriberInterface<? extends Message>>> subscribers = new java.util.ArrayList<>();
		  
		  RosTopicSubscriberInterface<? extends Message> sub=new RosIPABAPISubscriber();
		  java.util.Map.Entry<String,RosTopicSubscriberInterface<? extends Message>> pair=new java.util.AbstractMap.SimpleEntry<String,RosTopicSubscriberInterface<? extends Message>>(nameSpace+"/api_command", sub);
		  subscribers.add(pair);
			  
		  new ThePeoplesGloriousNetworkProcessor(rosUri, gfe_communicator, sensorCommunicator, ppsOffsetProvider, robotModel, nameSpace, tfPrefix, subscribers, null);

		  drcSimulationFactory = simulationStarter.getDRCSimulationFactory();
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
	   drcSimulationFactory.simulate();
   }
   
   public void loadEnvironment(String filename)
   {
	   System.out.println("Loading environment from '"+filename+"'");
	   environment.load(filename);
	   drcSimulationFactory.updateEnvironment(environment);
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


