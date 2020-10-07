package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class CoPTrajectoryVisualizer
{
   public static void visualize(CoPTrajectoryGenerator copTrajectoryGenerator)
   {
      YoRegistry registry = new YoRegistry("visualizer");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      WaypointViewer viewer = new WaypointViewer(registry, graphicsListRegistry);

      YoFramePoint3D desiredCoP = new YoFramePoint3D("desiredCoP", ReferenceFrame.getWorldFrame(), registry);
      BagOfBalls desiredCoPViz = new BagOfBalls(100, 0.005, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL, registry, graphicsListRegistry);

      SimulationConstructionSet simulationConstructionSet = new SimulationConstructionSet(new Robot("Dummy"));
      simulationConstructionSet.getRootRegistry().addChild(registry);
      simulationConstructionSet.addYoGraphicsListRegistry(graphicsListRegistry);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = simulationConstructionSet.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      simulationConstructionSet.startOnAThread();


      List<? extends ContactStateProvider> contactStateProviderList = copTrajectoryGenerator.getContactStateProviders();
      double totalDuration = Math.min(10.0, contactStateProviderList.get(contactStateProviderList.size() - 1).getTimeInterval().getEndTime());
      viewer.updateWaypoints(contactStateProviderList);

      for (double time = 0; time <= totalDuration; time += 0.01)
      {
         copTrajectoryGenerator.update(time, desiredCoP);
         desiredCoPViz.setBall(desiredCoP);
         simulationConstructionSet.tickAndUpdate();
      }

      ThreadTools.sleepForever();
   }

}
