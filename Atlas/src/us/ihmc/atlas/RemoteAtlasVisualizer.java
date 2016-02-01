package us.ihmc.atlas;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.visualization.GainControllerSliderBoard;
import us.ihmc.darpaRoboticsChallenge.visualization.WalkControllerSliderBoard;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizerStateListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class RemoteAtlasVisualizer implements SCSVisualizerStateListener
{
   private static final AtlasSliderBoardType defaultSliderBoardType = AtlasSliderBoardType.WALK_CONTROLLER;
   
   public enum AtlasSliderBoardType {GAIN_CONTROLLER, JOINT_ANGLE_OFFSET, WALK_CONTROLLER}

   private final DRCRobotModel drcRobotModel;

   public RemoteAtlasVisualizer(int bufferSize, DRCRobotModel drcRobotModel)
   {
      this.drcRobotModel = drcRobotModel;

      SCSVisualizer scsVisualizer = new SCSVisualizer(bufferSize);
      scsVisualizer.addSCSVisualizerStateListener(this);
      scsVisualizer.addButton("requestStop", 1.0);
      scsVisualizer.addButton("calibrateWristForceSensors", 1.0);
      scsVisualizer.setShowOverheadView(true);
      
      YoVariableClient client = new YoVariableClient(scsVisualizer, "remote");
      client.start();
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      switch (defaultSliderBoardType)
      {
         case WALK_CONTROLLER :
            new WalkControllerSliderBoard(scs, registry, null);

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
      
      try
      {
        DRCRobotModel.RobotTarget target = config.getBoolean(runningOnRealRobot.getID()) ? DRCRobotModel.RobotTarget.REAL_ROBOT : DRCRobotModel.RobotTarget.SCS;
        DRCRobotModel model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), target, false);

         new RemoteAtlasVisualizer(bufferSize, model);         
      }
      catch(IllegalArgumentException e)
      {
         System.err.println();
         System.err.println("Usage: java " + RemoteAtlasVisualizer.class.getName());
         System.err.println("                " + jsap.getUsage());
         System.err.println();
         System.exit(1);
      }
   }
}
