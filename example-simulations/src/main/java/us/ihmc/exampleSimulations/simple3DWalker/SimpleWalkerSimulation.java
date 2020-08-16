package us.ihmc.exampleSimulations.simple3DWalker;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import javax.swing.*;

public class SimpleWalkerSimulation
{
   private SimulationConstructionSet scs;
   private boolean withFeet = true;
   private boolean withInertiaControl = false;
   private boolean withImpactControl = false;
   private boolean withTwan = true;

   private boolean withPush = true;
   private double PUSH_FORCE_Y = 60;
   private double PUSH_DURATION = 0.2;

   private boolean withHeightOnly = false;
   private boolean withVelocityChange = false;
   private double DESIRED_VELOCITY = 1.0;


   SimpleWalkerSimulation() throws SimulationExceededMaximumTimeException
   {
      double simulationDT = 1e-4;
      
      SimpleWalkerRobot robot = new SimpleWalkerRobot(withFeet, false);
      YoRegistry registry = robot.getRobotsYoRegistry();


      SimpleWalkerController walkerController = new SimpleWalkerController(robot, simulationDT, withInertiaControl, withImpactControl, withTwan, withHeightOnly);
      robot.addYoGraphicsListRegistry(walkerController.getYoGraphicsListRegistry());
      robot.setController(walkerController);
    


      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      final SimulationOverheadPlotter simulationOverheadPlotter = new SimulationOverheadPlotter();


      scs = new SimulationConstructionSet(robot);
      scs.setDT(simulationDT, 10);
      scs.startOnAThread();

      simulationOverheadPlotter.setDrawHistory(true);

      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getDesiredCoPGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentCoPGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentCoMGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getDesiredICPGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentICPGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentLFootGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentRFootGraphicArtifact());
      //simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getFootArtifact());

      simulationOverheadPlotter.setXVariableToTrack((YoDouble) robot.findVariable("q_x"));
      simulationOverheadPlotter.setYVariableToTrack((YoDouble) robot.findVariable("q_y"));


      scs.attachPlaybackListener(simulationOverheadPlotter);
      JPanel simulationOverheadPlotterJPanel = simulationOverheadPlotter.getJPanel();
      scs.addExtraJpanel(simulationOverheadPlotterJPanel, "Plotter", true);
      JPanel plotterKeyJPanel = simulationOverheadPlotter.getJPanelKey();

      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);
      scs.addExtraJpanel(scrollPane, "Plotter Legend", false);

      yoGraphicsListRegistry.update();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, false);
      yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());

      scs.startOnAThread();

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs,100);


      if (withPush)
      {
         walkerController.setDesiredBodyVelocityX(DESIRED_VELOCITY);
         ExternalForcePoint externalForcePoint = new ExternalForcePoint("externalForce", robot);
         externalForcePoint.setForce(PUSH_FORCE_Y, 0, 0);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(6.1);
         robot.getRootJoints().get(0).addExternalForcePoint(externalForcePoint);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(PUSH_DURATION);
         robot.getRootJoints().get(0).removeExternalForcePoint(externalForcePoint);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(20.0);
      }

      if (withVelocityChange)
      {
         walkerController.setDesiredBodyVelocityX(DESIRED_VELOCITY);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(4.0);
         walkerController.setDesiredBodyVelocityX(0.0);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(3.0);
      }

/*
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoRegistry registry = new YoRegistry("fullwalker");
      SimpleWalkerRobotModel simpleWalkerRobotModel = new SimpleWalkerRobotModel();
      SimpleWalkerControllerForModel scsController = new SimpleWalkerControllerForModel(simpleWalkerRobotModel, 0.001);
      SCSRobotFromInverseDynamicsRobotModel scsRobot = simpleWalkerRobotModel.getSCSRobot();
      scsRobot.setController(scsController);
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      scs = new SimulationConstructionSet(scsRobot, parameters);
      scs.startOnAThread();
*/




   }
   
   public static void main(String[] args) throws SimulationExceededMaximumTimeException
   {
     new SimpleWalkerSimulation();
   }
}
