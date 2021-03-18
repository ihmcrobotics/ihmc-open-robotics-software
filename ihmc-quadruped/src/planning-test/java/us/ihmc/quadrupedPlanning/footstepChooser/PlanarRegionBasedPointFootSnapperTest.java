package us.ihmc.quadrupedPlanning.footstepChooser;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

import static us.ihmc.robotics.Assert.*;

public class PlanarRegionBasedPointFootSnapperTest
{
   private static final double epsilon = 1e-6;

   @Test
   public void testSnappingToFlatSquare()
   {
      PointFootSnapperParameters testParameters = new PointFootSnapperParameters()
      {
         @Override
         public double distanceInsidePlanarRegion()
         {
            return 0.1;
         }

         @Override
         public double maximumNormalAngleFromVertical()
         {
            return 1.0;
         }
      };

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(1.0, 2.0, -0.5);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);
      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();

      PlanarRegionBasedPointFootSnapper snapper = new PlanarRegionBasedPointFootSnapper(testParameters);
      snapper.setPlanarRegionsList(planarRegionsList);

      Point3DReadOnly snappedStep;

      snappedStep = snapper.snapStep(1.0, 2.0);
      assertTrue(snappedStep.epsilonEquals(new Point3D(1.0, 2.0, -0.5), epsilon));

      snappedStep = snapper.snapStep(1.1, 1.9);
      assertTrue(snappedStep.epsilonEquals(new Point3D(1.1, 1.9, -0.5), epsilon));

      snappedStep = snapper.snapStep(-0.1, 0.1);
      assertTrue(snappedStep.epsilonEquals(new Point3D(0.6, 1.6, -0.5), epsilon));

      snappedStep = snapper.snapStep(1.45, 2.25);
      assertTrue(snappedStep.epsilonEquals(new Point3D(1.4, 2.25, -0.5), epsilon));
   }

   @Test
   public void testSnappingToRotatedSquare()
   {
      PointFootSnapperParameters testParameters = new PointFootSnapperParameters()
      {
         @Override
         public double distanceInsidePlanarRegion()
         {
            return 0.1;
         }

         @Override
         public double maximumNormalAngleFromVertical()
         {
            return Math.PI;
         }
      };

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(-1.0, -1.0, 0.5);
      planarRegionsListGenerator.rotate(0.25 * Math.PI, Axis3D.X);
      planarRegionsListGenerator.addRectangle(2.0, 2.0);
      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();

      PlanarRegionBasedPointFootSnapper snapper = new PlanarRegionBasedPointFootSnapper(testParameters);
      snapper.setPlanarRegionsList(planarRegionsList);

      Point3DReadOnly snappedStep;

      snappedStep = snapper.snapStep(-1.0, -1.0);
      assertTrue(snappedStep.epsilonEquals(new Point3D(-1.0, -1.0, 0.5), epsilon));

      snappedStep = snapper.snapStep(-1.1, -1.1);
      assertTrue(snappedStep.epsilonEquals(new Point3D(-1.1, -1.1, 0.4), epsilon));

      snappedStep = snapper.snapStep(0.1, 0.1);
      assertTrue(snappedStep.epsilonEquals(new Point3D(- 0.1, -1.0 + 0.9 * Math.cos(0.25 * Math.PI), 0.5 + 0.9 * Math.sin(0.25 * Math.PI)), epsilon));

      snappedStep = snapper.snapStep(-1.95, -1.3);
      assertTrue(snappedStep.epsilonEquals(new Point3D(-1.9, -1.3, 0.5 - 0.3 * Math.tan(0.25 * Math.PI)), epsilon));
   }
}
