package us.ihmc.exampleSimulations.nonrewindableGraphicsExample;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.visualizers.SCSPointCloudVisualizer;

import javax.vecmath.Point3d;
import java.util.ArrayList;

public class PointCloudVisualizationSimulation
{
   public PointCloudVisualizationSimulation()
   {
      PointCloudRobot pointCloudRobot = new PointCloudRobot("PointCloudRobot");
      SimulationConstructionSet scs = new SimulationConstructionSet(pointCloudRobot);

      SCSPointCloudVisualizer pointCloudVisualizer = new SCSPointCloudVisualizer(scs);

      ArrayList<Point3d> points = new ArrayList<>();
      for(double i = 0; i < 1.0; i += 0.1)
      {
         for(double j = 0; j < 1.0; j += 0.1)
         {
            points.add(new Point3d(1.0, i, j));
         }
      }

      pointCloudVisualizer.addPoints(points);

      scs.startOnAThread();

      try
      {
         Thread.sleep(5000);
      }
      catch(InterruptedException e)
      {
         e.printStackTrace();
      }

      pointCloudVisualizer.clear();

      try
      {
         Thread.sleep(5000);
      }
      catch(InterruptedException e)
      {
         e.printStackTrace();
      }

      pointCloudVisualizer.addPoints(points);
   }

   public static void main(String[] args)
   {
      new PointCloudVisualizationSimulation();
   }
}
