package us.ihmc.jMonkeyEngineToolkit.jme;

import java.util.concurrent.LinkedBlockingQueue;

import com.jme3.scene.Node;

import us.ihmc.jMonkeyEngineToolkit.Graphics3DFrameListener;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DWorld;

public class JMEGraphics3DWorld extends Graphics3DWorld implements Graphics3DFrameListener
{
   LinkedBlockingQueue<Node> nodesToAddPostFrame = new LinkedBlockingQueue<>();
   
   public JMEGraphics3DWorld(String worldName, JMEGraphics3DAdapter jmeGraphics3dAdapter)
   {
      super(worldName, jmeGraphics3dAdapter);
   }

   public JMEGraphics3DWorld(JMEGraphics3DAdapter jmeGraphics3dAdapter)
   {
      this(JMEGraphics3DWorld.class.getSimpleName(), jmeGraphics3dAdapter);
   }
   
   @Override
   public JMEGraphics3DAdapter getGraphics3DAdapter()
   {
      return (JMEGraphics3DAdapter) super.getGraphics3DAdapter();
   }
   
   @Override
   protected void start()
   {
      super.start();
      
      addFrameListener(this);
   }
   
   public Node getJMERootNode()
   {
      return getGraphics3DAdapter().getRenderer().getZUpNode();
   }
   
   public void addChild(Node child)
   {
      if (viewportAdapter == null)
      {
         getGraphics3DAdapter().getRenderer().getZUpNode().attachChild(child);
      }
      else
      {
         nodesToAddPostFrame.add(child);
      }
   }
   
   @Override
   public void postFrame(double timePerFrame)
   {
      super.postFrame(timePerFrame);
      
      while (!nodesToAddPostFrame.isEmpty())
      {
         getGraphics3DAdapter().getRenderer().getZUpNode().attachChild(nodesToAddPostFrame.poll());
      }
   }
}
