package us.ihmc.exampleSimulations.nonrewindableGraphicsExample;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PointCloudVisualizationSimulation
{
   public PointCloudVisualizationSimulation()
   {
      PointCloudRobot pointCloudRobot = new PointCloudRobot("PointCloudRobot");
      SimulationConstructionSet scs = new SimulationConstructionSet(pointCloudRobot);

      Graphics3DObject testGraphicObject = new Graphics3DObject();
      testGraphicObject.addSphere(0.1, YoAppearance.Aqua());
      scs.addStaticLinkGraphics(testGraphicObject, Graphics3DNodeType.VISUALIZATION);

      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new PointCloudVisualizationSimulation();
   }
}
