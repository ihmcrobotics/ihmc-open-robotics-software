package us.ihmc.atlas;

import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.visualization.GainControllerSliderBoard;
import us.ihmc.darpaRoboticsChallenge.visualization.WalkControllerSliderBoard;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

public class RemoteAtlasVisualizer implements SCSVisualizerStateListener
{
   private final boolean showOverheadView = true;
   private static final AtlasSliderBoardType defaultSliderBoardType = AtlasSliderBoardType.WALK_CONTROLLER; 
   
   public enum AtlasSliderBoardType {GAIN_CONTROLLER, JOINT_ANGLE_OFFSET, WALK_CONTROLLER}
   
   private DRCRobotModel drcRobotModel;
   private SCSVisualizer scsVisualizer;
   
   public RemoteAtlasVisualizer(int bufferSize, DRCRobotModel drcRobotModel)
   {
      this.drcRobotModel = drcRobotModel;
      scsVisualizer = new SCSVisualizer(bufferSize);
      scsVisualizer.addSCSVisualizerStateListener(this);
      scsVisualizer.addButton("requestStop", 1.0);
      scsVisualizer.addButton("setWristForceSensorsToZero", 1.0);
      
      YoVariableClient client = new YoVariableClient(scsVisualizer, "remote", showOverheadView);
      client.start();
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      switch (defaultSliderBoardType)
      {
         case WALK_CONTROLLER :
            new WalkControllerSliderBoard(scs, registry);

            break;

         case GAIN_CONTROLLER :
            new GainControllerSliderBoard(scs, registry, drcRobotModel.getGeneralizedRobotModel());

            break;

         case JOINT_ANGLE_OFFSET :
            new JointAngleOffsetSliderBoard(scs, registry, drcRobotModel.getGeneralizedRobotModel());

            break;
      }
   }

   public static void main(String[] args) throws JSAPException
   {
      int bufferSize = 16384;
      JSAP jsap = new JSAP();
      
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      
      Switch runningOnRealRobot = new Switch("runningOnRealRobot").setLongFlag("realRobot");

      
      jsap.registerParameter(robotModel);
      jsap.registerParameter(runningOnRealRobot);

      JSAPResult config = jsap.parse(args);
      
      if (config.success())
      {
        AtlasTarget target = config.getBoolean(runningOnRealRobot.getID()) ? AtlasTarget.REAL_ROBOT : AtlasTarget.SIM;
        DRCRobotModel model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), target, false);

         new RemoteAtlasVisualizer(bufferSize, model);         
      }
      else
      {
         System.err.println();
         System.err.println("Usage: java " + RemoteAtlasVisualizer.class.getName());
         System.err.println("                " + jsap.getUsage());
         System.err.println();
         System.exit(1);
      }
   }
}
