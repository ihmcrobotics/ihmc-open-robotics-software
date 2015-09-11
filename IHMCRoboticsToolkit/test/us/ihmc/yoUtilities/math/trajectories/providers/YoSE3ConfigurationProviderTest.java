package us.ihmc.yoUtilities.math.trajectories.providers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.trajectories.providers.YoSE3ConfigurationProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

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

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, null);
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
      FrameOrientation orientationToPack = new FrameOrientation();
      provider.get(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FramePoint framePointToPack = new FramePoint();
      provider.get(framePointToPack);

      assertEquals(referenceFrame, framePointToPack.getReferenceFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
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
