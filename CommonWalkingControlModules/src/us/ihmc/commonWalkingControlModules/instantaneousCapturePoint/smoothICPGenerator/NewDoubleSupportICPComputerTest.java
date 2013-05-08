package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.test.JUnitTools;

public class NewDoubleSupportICPComputerTest
{
   @Test
   public void testComputeICPCornerPoints()
   {
      NewDoubleSupportICPComputer newDoubleSupportICPComputer = new NewDoubleSupportICPComputer();

      ArrayList<Point3d> footLocations = new ArrayList<Point3d>();
      footLocations.add(new Point3d(0.0, 0.0, 0.0));
      footLocations.add(new Point3d(1.0, 1.0, 0.0));
      footLocations.add(new Point3d(0.0, 2.0, 0.0));
      footLocations.add(new Point3d(1.0, 3.0, 0.0));

      double singleSupportDuration = 0.5;
      double doubleSupportDuration = 0.2;

      double omega0 = 0.75;

      int numberOfCornerPoints = footLocations.size() - 1;
      Point3d[] icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, footLocations,
                                     singleSupportDuration + doubleSupportDuration, omega0);

      assertEquals(numberOfCornerPoints, icpCornerPoints.length);

      for (int index = 0; index < numberOfCornerPoints - 1; index++)
      {
         Point3d footLocation = footLocations.get(index);
         Point3d firstCorner = icpCornerPoints[index];
         Point3d secondCorner = icpCornerPoints[index + 1];

         assertTrue(GeometryTools.arePointsInOrderAndColinear(footLocation, firstCorner, secondCorner, 1e-4));
      }

      assertTrue(GeometryTools.arePointsInOrderAndColinear(footLocations.get(numberOfCornerPoints - 1), icpCornerPoints[numberOfCornerPoints - 1],
              footLocations.get(numberOfCornerPoints), 1e-4));


   }


   @Test
   public void testComputeSingleSupportICP()
   {
      NewDoubleSupportICPComputer newDoubleSupportICPComputer = new NewDoubleSupportICPComputer();

      Point3d supportFoot = new Point3d(0.1, 0.2, 0.3);
      Point3d cornerPoint0 = new Point3d(0.3, 0.4, 0.5);

      double omega0 = 0.7;
      double singleSupportDuration = 0.7;
      double doubleSupportDuration = 1.1;

      double doubleSupportFirstStepFraction = 0.5;

      Point3d icpPosition = new Point3d();
      Vector3d icpVelocity = new Vector3d();

      Point3d singleSupportStartICP = newDoubleSupportICPComputer.computeSingleSupportStartICP(supportFoot, cornerPoint0, doubleSupportDuration,
                                         doubleSupportFirstStepFraction, omega0);
      Point3d singleSupportEndICP = newDoubleSupportICPComputer.computeSingleSupportEndICP(supportFoot, cornerPoint0, doubleSupportDuration,
                                       doubleSupportFirstStepFraction, singleSupportDuration, omega0);

      assertTrue(GeometryTools.arePointsInOrderAndColinear(supportFoot, cornerPoint0, singleSupportStartICP, 1e-4));
      assertTrue(GeometryTools.arePointsInOrderAndColinear(cornerPoint0, singleSupportStartICP, singleSupportEndICP, 1e-4));

      Point3d initialICPPosition = new Point3d(singleSupportStartICP);
      Point3d previousICPPosition = new Point3d(singleSupportStartICP);

      double deltaT = 0.001;
      for (double time = deltaT; time <= singleSupportDuration; time = time + deltaT)
      {
         newDoubleSupportICPComputer.computeSingleSupportICPPositionAndVelocity(icpPosition, icpVelocity, supportFoot, singleSupportStartICP, omega0, time);

         assertTrue(GeometryTools.arePointsInOrderAndColinear(supportFoot, initialICPPosition, icpPosition, 1e-4));

         Vector3d approximateVelocity = new Vector3d(icpPosition);
         approximateVelocity.sub(previousICPPosition);
         approximateVelocity.scale(1.0 / deltaT);

         JUnitTools.assertTuple3dEquals(approximateVelocity, icpVelocity, 1e-3);
         previousICPPosition.set(icpPosition);
      }

      JUnitTools.assertTuple3dEquals(singleSupportEndICP, icpPosition, 1e-3);
   }


}
