package us.ihmc.robotics.robotSide;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
public class RecyclingQuadrantDependentListTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testBasicMethods()
   {
      RecyclingQuadrantDependentList<FramePoint3D> recyclingQuadrantDependentList = new RecyclingQuadrantDependentList<>(FramePoint3D.class);

      FramePoint3D zeroPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         recyclingQuadrantDependentList.add(robotQuadrant);
         assertTrue("not equal", zeroPoint.epsilonEquals(recyclingQuadrantDependentList.get(robotQuadrant), 1e-7));
      }

      FramePoint3D framePoint = recyclingQuadrantDependentList.get(RobotQuadrant.FRONT_LEFT);
      recyclingQuadrantDependentList.remove(RobotQuadrant.FRONT_LEFT);
      assertEquals("not size 3", 3, recyclingQuadrantDependentList.size());

      recyclingQuadrantDependentList.add(RobotQuadrant.FRONT_LEFT);
      FramePoint3D framePoint2 = recyclingQuadrantDependentList.get(RobotQuadrant.FRONT_LEFT);
      assertTrue("not equal", framePoint2.epsilonEquals(framePoint, 1e-7));
      assertTrue("ref not equal", framePoint2 == framePoint);

      for (int i = 0; i < 4; i++)
      {
         recyclingQuadrantDependentList.get(RobotQuadrant.values[i]).add(i, 0.0, 0.0);
      }

      for (int j = 0; j < recyclingQuadrantDependentList.keys().length; j++)
      {
         FramePoint3D framePoint3 = recyclingQuadrantDependentList.keys()[j];
         assertEquals("not equal", j, framePoint3.getX(), 1e-7);
      }

      recyclingQuadrantDependentList.remove(RobotQuadrant.FRONT_LEFT);
      int k = 0;
      for (RobotQuadrant quadrant : recyclingQuadrantDependentList.quadrants())
      {
         recyclingQuadrantDependentList.get(quadrant).set(k - 6, 0.0, 0.0);
         ++k;
      }

      for (int j = 0; j < recyclingQuadrantDependentList.keys().length; j++)
      {
         FramePoint3D framePoint3 = recyclingQuadrantDependentList.keys()[j];
         assertEquals("not equal", j - 6, framePoint3.getX(), 1e-7);
      }
   }
}
