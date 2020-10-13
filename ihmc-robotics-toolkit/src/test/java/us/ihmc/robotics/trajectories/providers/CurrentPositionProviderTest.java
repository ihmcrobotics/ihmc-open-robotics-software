package us.ihmc.robotics.trajectories.providers;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;

public class CurrentPositionProviderTest
{
   private ReferenceFrame referenceFrame;
   private CurrentPositionProvider provider;

   @BeforeEach
   public void setUp()
   {
      referenceFrame = ReferenceFrameTools.constructARootFrame("rootNameTEST");
   }

   @AfterEach
   public void tearDown()
   {
      referenceFrame = null;
   }

	@Test
   public void testConstructor()
   {
      provider = new CurrentPositionProvider(null);
      provider = new CurrentPositionProvider(referenceFrame);
   }

	@Test
   public void testGet()
   {
      provider = new CurrentPositionProvider(referenceFrame);
      FramePoint3D framePointToPack = new FramePoint3D(referenceFrame);

      provider.getPosition(framePointToPack);

      assertEquals(referenceFrame, framePointToPack.getReferenceFrame());
   }
}
