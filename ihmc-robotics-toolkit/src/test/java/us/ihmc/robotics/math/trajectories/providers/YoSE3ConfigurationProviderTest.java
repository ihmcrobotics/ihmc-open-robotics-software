package us.ihmc.robotics.math.trajectories.providers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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
      FrameQuaternion orientationToPack = new FrameQuaternion();
      provider.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FramePoint3D framePointToPack = new FramePoint3D();
      provider.getPosition(framePointToPack);

      assertEquals(referenceFrame, framePointToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetPose()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
      FramePose3D framePose;
      try
      {
         framePose = new FramePose3D();
         provider.setPose(framePose);
         fail();
      }
      catch (RuntimeException rte)
      {
      }

      framePose = new FramePose3D(referenceFrame);

      provider.setPose(framePose);
   }
}
