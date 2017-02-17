package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.util.concurrent.Callable;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;

public class JMELidarScanVisualizer extends JMEPointCloudVisualizer
{
   private Graphics3DNode lidarNode;

   public JMELidarScanVisualizer()
   {
      super(JMELidarScanVisualizer.class.getSimpleName());

      addLidarNode();
   }

   private void addLidarNode()
   {
      lidarNode = new Graphics3DNode("lidar", new Graphics3DObject());
      lidarNode.getGraphics3DObject().addModelFile("models/hokuyo.dae", YoAppearance.Black());
      lidarNode.getGraphics3DObject().addCoordinateSystem(1.0);

      addChild(lidarNode);
   }

   public void updateLidarNodeTransform(final RigidBodyTransform lidarNodeTransform)
   {
      getGraphics3DAdapter().getRenderer().enqueue(new Callable<Object>()
      {
         @Override
         public Object call() throws Exception
         {
            lidarNode.setTransform(lidarNodeTransform);
            
            return null;
         }
      });
   }

   public void updateLidarNodeTransform(final AffineTransform lidarNodeTransform)
   {
      getGraphics3DAdapter().getRenderer().enqueue(new Callable<Object>()
      {
         @Override
         public Object call() throws Exception
         {
            lidarNode.setTransform(lidarNodeTransform);
            
            return null;
         }
      });
   }
}
