package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;



import javax.swing.JPanel;
import javax.swing.JScrollPane;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.plotting.SimulationOverheadPlotter;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class SCSDoubleSupportICPTester
{

   public static void main(String[] args)
   {
      Robot testRobot = new Robot("testRobot");
      SimulationConstructionSet scs = new SimulationConstructionSet(testRobot);
      scs.setDT(1e-3, 1);
      scs.changeBufferSize(16000);
      scs.setPlaybackRealTimeRate(0.2);
      scs.setPlaybackDesiredFrameRate(0.001);
      SimulationOverheadPlotter simulationOverheadPlotter = new SimulationOverheadPlotter();
      simulationOverheadPlotter.setDrawHistory(false);

      scs.attachPlaybackListener(simulationOverheadPlotter);
      JPanel simulationOverheadPlotterJPanel = simulationOverheadPlotter.getJPanel();
      String plotterName = "Plotter";
      scs.addExtraJpanel(simulationOverheadPlotterJPanel, plotterName);
      JPanel plotterKeyJPanel = simulationOverheadPlotter.getJPanelKey();

      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);
      scs.addExtraJpanel(scrollPane, "Plotter Legend");

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      scs.getStandardSimulationGUI().selectPanel(plotterName);


      double singleSupportTime = 0.6;
      double doubleSupportTime = 0.2;
      double initialTransferSupportTime = 0.4; 
      double steppingTime = singleSupportTime + doubleSupportTime;
      RobotController controller = new SCSDoubleSupportICPTesterController5(dynamicGraphicObjectsListRegistry, testRobot.getYoTime(), scs.getDT(),
            singleSupportTime, doubleSupportTime, initialTransferSupportTime, testRobot);
      controller.initialize();

      testRobot.setController(controller, 1); // Jojo: was 5 before

      dynamicGraphicObjectsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

      scs.startOnAThread();
      scs.simulate(7.5 * steppingTime);
   }
}

