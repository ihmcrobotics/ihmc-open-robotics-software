package us.ihmc.footstepPlanning.polygonWiggling;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.polygonWiggling.LegCollisionConstraintCalculator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class LegCollisionConstraintCalculatorTest
{
   private boolean visualize = true;
   private SimulationConstructionSet scs;
   private YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;
   private LegCollisionConstraintCalculator collisionConstraintCalculator;

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("dummy"));
         scs.setGroundVisible(false);
         collisionConstraintCalculator = new LegCollisionConstraintCalculator(graphicsListRegistry, scs.getRootRegistry());
         yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("PlanarRegions", 150, 100, scs.getRootRegistry());
         graphicsListRegistry.registerYoGraphic("PlanarRegionsGraphic", yoGraphicPlanarRegionsList);
         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.startOnAThread();
      }
      else
      {
         collisionConstraintCalculator = new LegCollisionConstraintCalculator();
      }

      double shinPitch = Math.toRadians(25.0);
      double shinRadius = 0.05;
      double shinLength = 0.4;
      Cylinder3D legCylinder = new Cylinder3D(shinLength, shinRadius);

      RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
      transformGenerator.translate(0.0, 0.0, 0.05);
      transformGenerator.rotate(shinPitch, Axis3D.Y);
      transformGenerator.translate(0.0, 0.0, 0.5 * shinLength);
      RigidBodyTransform transformToSoleFrame = transformGenerator.getRigidBodyTransformCopy();
      collisionConstraintCalculator.setLegCollisionShape(legCylinder, transformToSoleFrame);
   }

   @Test
   @Disabled
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
      PlanarRegion firstStep = stairCaseRegions.getPlanarRegion(1);

      visualizePlanarRegions(stairCaseRegions);
      collisionConstraintCalculator.calculateLegCollisionGradient(new RigidBodyTransform(), firstStep.getTransformToWorld(), stairCaseRegions, new Vector3D());

      if (visualize)
      {
         scs.tickAndUpdate();
         scs.cropBuffer();
         ThreadTools.sleepForever();
      }
   }

   private void visualizePlanarRegions(PlanarRegionsList planarRegionsList)
   {
      if (!visualize)
      {
         return;
      }

      yoGraphicPlanarRegionsList.clear();
      yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(planarRegionsList);
      for (int i = 0; i < 100; i++)
      {
         yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
         scs.tickAndUpdate();
      }
   }
}
