package us.ihmc.robotics.math.trajectories.providers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoSE3ConfigurationProviderTest
{
   private String name = "nameTest";

   private ReferenceFrame referenceFrame;
   private YoVariableRegistry registry;

   private YoSE3ConfigurationProvider provider;

   @Before
   public void setUp()
   {
      referenceFrame = ReferenceFrame.constructARootFrame("rootNameTEST");
      registry = new YoVariableRegistry("registryTEST");
   }

   @After
   public void tearDown()
   {
      referenceFrame = null;
      registry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, null);
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
      FrameOrientation orientationToPack = new FrameOrientation();
      provider.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FramePoint framePointToPack = new FramePoint();
      provider.getPosition(framePointToPack);

      assertEquals(referenceFrame, framePointToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetPose()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
      FramePose framePose;
      try
      {
         framePose = new FramePose();
         provider.setPose(framePose);
         fail();
      }
      catch (RuntimeException rte)
      {
      }

      framePose = new FramePose(referenceFrame);

      provider.setPose(framePose);
   }
}
