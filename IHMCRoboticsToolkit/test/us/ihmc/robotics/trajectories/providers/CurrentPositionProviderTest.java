package us.ihmc.robotics.trajectories.providers;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import static org.junit.Assert.assertEquals;

public class CurrentPositionProviderTest
{
   private ReferenceFrame referenceFrame;
   private CurrentPositionProvider provider;

   @Before
   public void setUp()
   {
      referenceFrame = ReferenceFrame.constructARootFrame("rootNameTEST");
   }

   @After
   public void tearDown()
   {
      referenceFrame = null;
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      provider = new CurrentPositionProvider(null);
      provider = new CurrentPositionProvider(referenceFrame);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet()
   {
      provider = new CurrentPositionProvider(referenceFrame);
      FramePoint framePointToPack = new FramePoint(referenceFrame);

      provider.get(framePointToPack);

      assertEquals(referenceFrame, framePointToPack.getReferenceFrame());
   }
}
