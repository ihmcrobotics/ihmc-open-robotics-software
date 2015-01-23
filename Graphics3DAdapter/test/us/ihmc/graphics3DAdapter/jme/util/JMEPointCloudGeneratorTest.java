 package us.ihmc.graphics3DAdapter.jme.util;

import java.util.Random;

import us.ihmc.graphics3DAdapter.Graphics3DWorld;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.jme.JMEGraphics3DAdapter;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.lidar.polarLidar.SparseLidarScan;
import us.ihmc.utilities.lidar.polarLidar.geometry.LidarScanParameters;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

import com.jme3.scene.Node;

public class JMEPointCloudGeneratorTest
{
   public static void main(String[] args)
   {
      new JMEPointCloudGeneratorTest().testJmePointCloudGenerator();
   }
   
   public void testJmePointCloudGenerator()
   {
      JMEGraphics3DAdapter jmeGraphics3DAdapter = new JMEGraphics3DAdapter(false);
      
      Graphics3DWorld world = new Graphics3DWorld(jmeGraphics3DAdapter);
      
      Graphics3DObject coordinateFrameObject = new Graphics3DObject();
      coordinateFrameObject.addCoordinateSystem(1.0);
      
      Graphics3DNode coordinateFrameNode = new Graphics3DNode(getClass().getSimpleName() + "CoordinateFrame", coordinateFrameObject);
      
      world.addChild(coordinateFrameNode);
      
      Random random = new Random();
      
      int numPoints = 1000;
      
      float[] ranges = RandomTools.generateRandomFloatArray(random, numPoints, 5.0f);
      
      int[] indexes = new int[numPoints];
      
      for (int i = 0; i < indexes.length; i++)
      {
         indexes[i] = i;
      }
      
      LidarScanParameters lidarScanParameters = new LidarScanParameters(numPoints, 2* Math.PI, 0, Double.POSITIVE_INFINITY);
      
      RigidBodyTransform startTransform = new RigidBodyTransform();
      RigidBodyTransform endTransform = new RigidBodyTransform();
      
      endTransform.rotX(Math.PI / 4);
      
      SparseLidarScan sparseLidarScan = new SparseLidarScan(1, lidarScanParameters, startTransform, endTransform, indexes, ranges);
      
      JMEPointCloudGenerator jmePointCloudGenerator = new JMEPointCloudGenerator(jmeGraphics3DAdapter.getRenderer().getAssetManager());
      
      Node pointCloudNode = jmePointCloudGenerator.generatePointCloudGraph(sparseLidarScan.getAllPoints3f());
      
      jmeGraphics3DAdapter.getRenderer().getZUpNode().attachChild(pointCloudNode);
      
      world.startWithGui();
   }
}
