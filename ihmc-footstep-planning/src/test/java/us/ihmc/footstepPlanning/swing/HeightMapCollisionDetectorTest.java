package us.ihmc.footstepPlanning.swing;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointVectorNN;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class HeightMapCollisionDetectorTest
{
   @Test
   public void testPointOnBoxFromData()
   {
      FrameBox3D box = new FrameBox3D();
      box.getPosition().set(0.03377004675861024, 1.828394989015923E-12, 0.24781942420481712);
      box.getSize().set(0.2915384615384616, 0.15076923076923077, 0.3966666666666667);

      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.15, -0.06, 0.05);
      Point3DReadOnly pointOnBox = HeightMapCollisionDetector.getPointOnBoxWhenTheWholeBottomPenetrates(point, box);

      assertEquals(0.0, box.distance(pointOnBox), 1e-5);
   }

   @Test
   public void testGetLowestHeightOnBoxAtPoint()
   {
      // lowest point on box should be 0.0 for everywhere contained inside the box
      Box3D simpleBox = new Box3D(0.5, 0.5, 0.5);
      simpleBox.getPose().getShapePosition().set(0.0, 0.0, 0.25);

      Random random = new Random((1738L));
      for (int i = 0; i < 500; i++)
      {
         double x = RandomNumbers.nextDouble(random, 0.25);
         double y = RandomNumbers.nextDouble(random, 0.25);
         double z = HeightMapCollisionDetector.getLowestHeightOnBoxAtPoint(simpleBox, x, y);
         assertEquals("Failed at (x,y) = (" + x + ", " + y + ")", 0.0, z, 1e-10);
      }
      // lowest point on box should be NaN for everywhere contained outside the box
      for (int i = 0; i < 500; i++)
      {
         double x = RandomNumbers.nextDouble(random, 0.25, 0.25);
         double y = RandomNumbers.nextDouble(random, 0.25);
         double z = HeightMapCollisionDetector.getLowestHeightOnBoxAtPoint(simpleBox, x, y);
         assertEquals("Failed at (x,y) = (" + x + ", " + y + ")", 0.0, z, 1e-10);
      }
   }

   @Test
   public void testPointOnBox()
   {
      // lowest point on box should be 0.0 for everywhere contained inside the box
      Box3D simpleBox = new Box3D(0.5, 0.5, 0.5);
      simpleBox.getPose().getShapePosition().set(0.0, 0.0, 0.25);

      Point3D point = new Point3D(0.05, -0.03, 0.125);
      Point3D pointOnBox = new Point3D(0.05, -0.03, 0.125);
      Vector3D normalOnBox = new Vector3D(0.05, -0.03, 0.125);
      Point3D pointOnBoxExpected = new Point3D(0.05, -0.03, 0.0);
      HeightMapCollisionDetector.getPointOnBoxThatsPenetrating(simpleBox, point, pointOnBox, normalOnBox);

      EuclidCoreTestTools.assertEquals(pointOnBoxExpected, pointOnBox, 1e-10);
   }

}
