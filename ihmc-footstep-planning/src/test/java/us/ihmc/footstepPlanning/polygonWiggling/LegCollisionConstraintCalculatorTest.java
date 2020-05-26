package us.ihmc.footstepPlanning.polygonWiggling;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.polygonWiggling.LegCollisionConstraintCalculator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class LegCollisionConstraintCalculatorTest
{
   @Test
   public void testLegCollisionDetector()
   {
      double stepHeight = 0.3;
      double stepLength = 0.25;
      double stepWidth = 0.8;
      int steps = 4;

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(4.0, 4.0);
      planarRegionsListGenerator.translate(0.5 * stepLength, 0.0, 0.0);

      for (int i = 0; i < steps; i++)
      {
         planarRegionsListGenerator.translate(stepLength, 0.0, stepHeight);
         planarRegionsListGenerator.addRectangle(stepLength, stepWidth);
      }

      PlanarRegionsList stairCaseRegions = planarRegionsListGenerator.getPlanarRegionsList();
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));
      scs.setGroundVisible(false);

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      LegCollisionConstraintCalculator collisionConstraintCalculator = new LegCollisionConstraintCalculator(graphicsListRegistry, scs.getRootRegistry());

      double shinPitch = Math.toRadians(25.0);
      double shinRadius = 0.05;
      double shinLength = 0.4;

      PlanarRegion firstStep = stairCaseRegions.getPlanarRegion(1);

      Cylinder3D legCylinder = new Cylinder3D(shinLength, shinRadius);

      RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
      transformGenerator.translate(0.0, 0.0, 0.05);
      transformGenerator.rotate(shinPitch, Axis3D.Y);
      transformGenerator.translate(0.0, 0.0, 0.5 * shinLength);
      RigidBodyTransform transformToSoleFrame = transformGenerator.getRigidBodyTransformCopy();
      System.out.println(transformToSoleFrame);

      collisionConstraintCalculator.setLegCollisionShape(legCylinder, transformToSoleFrame);

      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.startOnAThread();

      collisionConstraintCalculator.calculateLegCollisionGradient(new Pose2D(), firstStep.getTransformToWorld(), stairCaseRegions, new Vector3D());

      for (int i = 0; i < 100; i++)
      {
         collisionConstraintCalculator.update();
         scs.tickAndUpdate();
      }

      ThreadTools.sleepForever();
   }
}
