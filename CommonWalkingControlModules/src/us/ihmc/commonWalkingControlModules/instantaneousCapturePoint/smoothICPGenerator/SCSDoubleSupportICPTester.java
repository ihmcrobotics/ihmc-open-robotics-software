package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

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

      PointAndLinePlotter pointAndLinePlotter = new PointAndLinePlotter(testRobot.getRobotsYoVariableRegistry());

      double singleSupportTime = 0.6;
      double doubleSupportTime = 0.2;
      double initialTransferSupportTime = 0.4;
      double steppingTime = singleSupportTime + doubleSupportTime;
      SCSDoubleSupportICPTesterController5 controller = new SCSDoubleSupportICPTesterController5(pointAndLinePlotter, testRobot.getYoTime(), scs.getDT(),
                                      singleSupportTime, doubleSupportTime, initialTransferSupportTime);
      controller.initialize();

      
      pointAndLinePlotter.createAndShowOverheadPlotterInSCS(scs);

      
      testRobot.setController(controller, 1);    // Jojo: was 5 before

      pointAndLinePlotter.addGraphicObjectsAndArtifactsToSCS(scs);

      scs.startOnAThread();
      scs.simulate(6.5 * steppingTime);
   }
}
