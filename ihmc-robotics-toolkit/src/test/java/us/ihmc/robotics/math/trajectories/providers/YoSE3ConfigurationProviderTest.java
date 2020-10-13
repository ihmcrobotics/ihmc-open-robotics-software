package us.ihmc.robotics.math.trajectories.providers;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSE3ConfigurationProviderTest
{
   private String name = "nameTest";

   private ReferenceFrame referenceFrame;
   private YoRegistry registry;

   private YoSE3ConfigurationProvider provider;

   @BeforeEach
   public void setUp()
   {
      referenceFrame = ReferenceFrameTools.constructARootFrame("rootNameTEST");
      registry = new YoRegistry("registryTEST");
   }

   @AfterEach
   public void tearDown()
   {
      referenceFrame = null;
      registry = null;
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testConstructor()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, null);
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
   }

	@Test
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

	@Test
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
