package us.ihmc.footstepPlanning.bodyPath;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.ArrayList;
import java.util.List;

public class AStarBodyPathSmootherTest
{
   @Test
   public void testSimpleSmoothing0()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      AStarBodyPathSmoother smoother = new AStarBodyPathSmoother(scs, graphicsListRegistry, scs.getRootRegistry());
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.startOnAThread();

      List<BodyPathLatticePoint> bodyPath = new ArrayList<>();
      bodyPath.add(new BodyPathLatticePoint(0, 0));
      bodyPath.add(new BodyPathLatticePoint(1, 0));
      bodyPath.add(new BodyPathLatticePoint(2, -1));
      bodyPath.add(new BodyPathLatticePoint(3, -1));

      smoother.doSmoothing(bodyPath);
      ThreadTools.sleepForever();
   }
}
