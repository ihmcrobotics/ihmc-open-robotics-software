package us.ihmc.ihmcPerception;

import us.ihmc.ihmcPerception.depthData.PointCloudDataReceiverInterface;
import us.ihmc.ihmcPerception.depthData.PointCloudSource;
import us.ihmc.ihmcPerception.depthData.RosPointCloudReceiver;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DWorld;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMELidarSpriteGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.utilities.ros.RosMainNode;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import java.util.ArrayList;

public class RosPointCloudVisualizer
{
   private final JMELidarSpriteGenerator lidarSpriteGenerator;
   private final RigidBodyTransform sensorTransform;
   private final RosPointCloudReceiver pointCloudReceiver;
   private final RosMainNode rosMainNode;

   public RosPointCloudVisualizer(String topicName, RosMainNode rosMainNode)
   {
      this(topicName, rosMainNode, null);
   }


   public RosPointCloudVisualizer(String topicName, RosMainNode rosMainNode, JMERenderer renderer)
   {
      this(topicName, rosMainNode, renderer, null);
   }

   public RosPointCloudVisualizer(String topicName, RosMainNode rosMainNode, JMERenderer renderer, RigidBodyTransform sensorTransform)
   {
      if(renderer == null)
      {
         JMEGraphics3DWorld pointCloudVisualizer = new JMEGraphics3DWorld("RosPointCloudVisualizer", new JMEGraphics3DAdapter(false));
         pointCloudVisualizer.startWithGui();

         renderer = pointCloudVisualizer.getGraphics3DAdapter().getRenderer();
      }

      this.rosMainNode = rosMainNode;
      this.sensorTransform = sensorTransform;

      lidarSpriteGenerator = new JMELidarSpriteGenerator(renderer);
      renderer.getZUpNode().attachChild(lidarSpriteGenerator);
      renderer.registerUpdatable(lidarSpriteGenerator);

      pointCloudReceiver = new RosPointCloudReceiver(topicName, rosMainNode, ReferenceFrame.getWorldFrame(),
            ReferenceFrame.getWorldFrame(), new PointCloudReceiver(), PointCloudSource.NEARSCAN);
   }

   public void detach()
   {
      lidarSpriteGenerator.clear();
      rosMainNode.removeSubscriber(pointCloudReceiver);
   }

   class PointCloudReceiver implements PointCloudDataReceiverInterface
   {
      @Override
      public void receivedPointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3d> points,
            PointCloudSource... sources)
      {
         Point3f[] point3fs = new Point3f[points.size()];
         for(int i = 0; i < points.size(); i++)
         {
            Point3d point = points.get(i);
            if(sensorTransform != null)
               sensorTransform.transform(point);
            point3fs[i] = new Point3f(point);
         }

         lidarSpriteGenerator.updatePoints(point3fs);
      }
   }
}
