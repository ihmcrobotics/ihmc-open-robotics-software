package us.ihmc.graphics3DAdapter.jme;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.examples.Graphics3DAdapterExampleOne;

public class JMEGraphics3dAdapterTest
{
   @Test(timeout=300000)
   public void testSimpleObject()
   {
      JMEGraphics3DAdapter renderer = new JMEGraphics3DAdapter();
      Graphics3DAdapterExampleOne example1 = new Graphics3DAdapterExampleOne();

      assertTrue(example1.doExample(renderer));
   }

}
