 package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.util.Random;

import com.jme3.scene.Node;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DWorld;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.lidar.LidarTestParameters;
import us.ihmc.jMonkeyEngineToolkit.jme.lidar.LidarTestScan;

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
      
      float[] ranges = RandomNumbers.nextFloatArray(random, numPoints, 5.0f);
      
      LidarTestParameters lidarScanParameters = new LidarTestParameters();//numPoints, 2* Math.PI, 0, Double.POSITIVE_INFINITY);
      lidarScanParameters.setScansPerSweep(numPoints);
      lidarScanParameters.setLidarSweepStartAngle(-Math.PI);
      lidarScanParameters.setLidarSweepEndAngle(Math.PI);
      lidarScanParameters.setMinRange(0.0);
      lidarScanParameters.setMaxRange(Double.POSITIVE_INFINITY);
      
      RigidBodyTransform startTransform = new RigidBodyTransform();
      RigidBodyTransform endTransform = new RigidBodyTransform();
      
      endTransform.setRotationRollAndZeroTranslation(Math.PI / 4);
      LidarTestScan lidarScan = new LidarTestScan(lidarScanParameters, startTransform, endTransform, ranges, 1);
      
      JMEPointCloudGenerator jmePointCloudGenerator = new JMEPointCloudGenerator(jmeGraphics3DAdapter.getRenderer().getAssetManager());
      
      Node pointCloudNode = jmePointCloudGenerator.generatePointCloudGraph(lidarScan.getAllPoints3f());
      
      jmeGraphics3DAdapter.getRenderer().getZUpNode().attachChild(pointCloudNode);
      
      world.startWithGui();
   }
}
