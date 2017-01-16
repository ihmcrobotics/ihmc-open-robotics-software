package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.util.Collection;
import java.util.concurrent.LinkedBlockingQueue;

import javax.vecmath.Tuple3f;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DWorld;

import com.jme3.scene.Node;

public class JMEPointCloudVisualizer extends JMEGraphics3DWorld
{
   JMEPointCloudGenerator jmePointCloudGenerator;
   LinkedBlockingQueue<Node> nodesToAddPostFrame = new LinkedBlockingQueue<>();
   
   protected JMEPointCloudVisualizer(String name)
   {
      super(name, new JMEGraphics3DAdapter(false));
      
      jmePointCloudGenerator = new JMEPointCloudGenerator(getGraphics3DAdapter().getRenderer().getAssetManager());
      
//      startWithGui(1000, 800);
      startWithoutGui();
      
      addCoordinateFrame();
   }
   
   public JMEPointCloudVisualizer()
   {
      this(JMEPointCloudVisualizer.class.getSimpleName());
   }
   
   public void addCoordinateFrame()
   {
      Graphics3DObject coordinateFrameObject = new Graphics3DObject();
      coordinateFrameObject.addCoordinateSystem(1.0);
      
      addChild(new Graphics3DNode("CoordinateFrameNode", coordinateFrameObject));
   }
   
   public <T extends Tuple3f> void addPointCloud(Collection<T> worldPoints)
   {
      Node pointCloud = jmePointCloudGenerator.generatePointCloudGraph(worldPoints);
    
      addChild(pointCloud);
   }
}
