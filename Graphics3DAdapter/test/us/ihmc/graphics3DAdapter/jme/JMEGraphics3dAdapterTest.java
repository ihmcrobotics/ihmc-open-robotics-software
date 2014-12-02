package us.ihmc.graphics3DAdapter.jme;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.examples.Graphics3DAdapterExampleOne;

public class JMEGraphics3dAdapterTest
{
   @Test
   public void testSimpleObject()
   {
      JMEGraphics3DAdapter renderer = new JMEGraphics3DAdapter();
      Graphics3DAdapterExampleOne example1 = new Graphics3DAdapterExampleOne();

      example1.doExample(renderer);
   }

}
