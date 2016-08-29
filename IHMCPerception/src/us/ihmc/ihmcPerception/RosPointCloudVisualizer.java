package us.ihmc.ihmcPerception;

import us.ihmc.graphics3DAdapter.jme.JMEGraphics3DAdapter;
import us.ihmc.graphics3DAdapter.jme.JMEGraphics3DWorld;
import us.ihmc.graphics3DAdapter.jme.JMERenderer;
import us.ihmc.graphics3DAdapter.jme.util.JMELidarSpriteGenerator;
import us.ihmc.ihmcPerception.depthData.PointCloudDataReceiverInterface;
import us.ihmc.ihmcPerception.depthData.PointCloudSource;
import us.ihmc.ihmcPerception.depthData.RosPointCloudReceiver;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.utilities.ros.RosMainNode;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.net.URI;
import java.util.ArrayList;

public class RosPointCloudVisualizer
{
   private final JMELidarSpriteGenerator lidarSpriteGenerator;

   private final RigidBodyTransform magicTransform = new RigidBodyTransform();
   {
      // This is to correct the differences between default reference frames in ROS and JME
      Quat4d magicRotation = new Quat4d();
      RotationTools.convertYawPitchRollToQuaternion(-Math.PI/2.0, 0.0, -Math.PI/2.0, magicRotation);
      magicTransform.set(magicRotation, new Vector3d());
   }

   public RosPointCloudVisualizer(String topicName, RosMainNode rosMainNode)
   {
      this(topicName, rosMainNode, null);
   }

   public RosPointCloudVisualizer(String topicName, RosMainNode rosMainNode, JMERenderer renderer)
   {
      if(renderer == null)
      {
         JMEGraphics3DWorld pointCloudVisualizer = new JMEGraphics3DWorld("RosPointCloudVisualizer", new JMEGraphics3DAdapter(false));
         pointCloudVisualizer.startWithGui();

         renderer = pointCloudVisualizer.getGraphics3DAdapter().getRenderer();
      }

      lidarSpriteGenerator = new JMELidarSpriteGenerator(renderer);
      renderer.getZUpNode().attachChild(lidarSpriteGenerator);
      renderer.registerUpdatable(lidarSpriteGenerator);

      new RosPointCloudReceiver(topicName, rosMainNode, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), new PointCloudReceiver(),
            PointCloudSource.NEARSCAN);
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
            magicTransform.transform(point);
            point3fs[i] = new Point3f(point);
         }

         lidarSpriteGenerator.updatePoints(point3fs);
      }
   }
}
