package us.ihmc.footstepPlanning.swing;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

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
}
