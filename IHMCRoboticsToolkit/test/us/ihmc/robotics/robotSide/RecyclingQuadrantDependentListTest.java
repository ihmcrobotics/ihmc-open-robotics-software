package us.ihmc.robotics.robotSide;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class RecyclingQuadrantDependentListTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBasicMethods()
   {
      RecyclingQuadrantDependentList<FramePoint> recyclingQuadrantDependentList = new RecyclingQuadrantDependentList<>(FramePoint.class);
      
      FramePoint zeroPoint = new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         recyclingQuadrantDependentList.add(robotQuadrant);
         assertTrue("not equal", zeroPoint.epsilonEquals(recyclingQuadrantDependentList.get(robotQuadrant), 1e-7));
      }
      
      FramePoint framePoint = recyclingQuadrantDependentList.get(RobotQuadrant.FRONT_LEFT);
      recyclingQuadrantDependentList.remove(RobotQuadrant.FRONT_LEFT);
      assertEquals("not size 3", 3, recyclingQuadrantDependentList.size());
      
      recyclingQuadrantDependentList.add(RobotQuadrant.FRONT_LEFT);
      FramePoint framePoint2 = recyclingQuadrantDependentList.get(RobotQuadrant.FRONT_LEFT);
      assertTrue("not equal", framePoint2.epsilonEquals(framePoint, 1e-7));
      assertTrue("ref not equal", framePoint2 == framePoint);
      
      for (int i = 0; i < 4; i++)
      {
         recyclingQuadrantDependentList.get(RobotQuadrant.values[i]).add(i, 0.0, 0.0);
      }
      
      for (int j = 0; j < recyclingQuadrantDependentList.values().length; j++)
      {
         FramePoint framePoint3 = recyclingQuadrantDependentList.values()[j];
         assertEquals("not equal", j, framePoint3.getX(), 1e-7);
      }
      
      recyclingQuadrantDependentList.remove(RobotQuadrant.FRONT_LEFT);
      int k = 0;
      for (RobotQuadrant quadrant : recyclingQuadrantDependentList.quadrants())
      {
         recyclingQuadrantDependentList.get(quadrant).set(k - 6, 0.0, 0.0);
         ++k;
      }
      
      for (int j = 0; j < recyclingQuadrantDependentList.values().length; j++)
      {
         FramePoint framePoint3 = recyclingQuadrantDependentList.values()[j];
         assertEquals("not equal", j - 6, framePoint3.getX(), 1e-7);
      }
   }
}
