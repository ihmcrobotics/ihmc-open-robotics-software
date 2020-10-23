package us.ihmc.robotics.math.trajectories.providers;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertSame;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;


public class YoPositionProviderTest
{
   private static final double EPSILON = 1e-10;
   
   private YoFramePoint3D yoFramePoint;
   private String namePrefix = "namePrefixTEST";
   private ReferenceFrame referenceFrame;
   private YoRegistry registry;
   
   private YoPositionProvider provider;
   
   private static double xValue = Math.random();
   private static double yValue = Math.random();
   private static double zValue = Math.random();
   
   @BeforeEach
   public void setUp()
   {
      referenceFrame = ReferenceFrameTools.constructARootFrame("rootNameTEST");
      registry = new YoRegistry("parentRegistryTEST");
      yoFramePoint = new YoFramePoint3D(namePrefix, referenceFrame, registry);
      yoFramePoint.set(xValue, yValue, zValue);
   }
   
   @AfterEach
   public void tearDown()
   {
      referenceFrame = null;
      registry = null;
      yoFramePoint = null;
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testConstructor()
   {
      provider = new YoPositionProvider(null);
      provider = new YoPositionProvider(yoFramePoint);
   }

	@Test
   public void testSetAndGet()
   {
      provider = new YoPositionProvider(yoFramePoint);
      
      FramePoint3D result = new FramePoint3D(referenceFrame);
      result.set(5.0, 10.0, 15.0);
      provider.set(result);
      
      FramePoint3D positionToPack = new FramePoint3D();
      provider.getPosition(positionToPack);
      
      assertSame(referenceFrame, positionToPack.getReferenceFrame());
      assertEquals(5.0, positionToPack.getX(), EPSILON);
      assertEquals(10.0, positionToPack.getY(), EPSILON);
      assertEquals(15.0, positionToPack.getZ(), EPSILON);
   }
}
