package us.ihmc.perception;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.perception.slamWrapper.SlamWrapper;
import us.ihmc.perception.slamWrapper.SlamWrapperNativeLibrary;
import us.ihmc.perception.tools.PerceptionEuclidTools;

public class SlamWrapperTest
{
   @Test
   public void testNativeSlamWrapperLibrary()
   {
      SlamWrapperNativeLibrary.load();

      SlamWrapper.FactorGraphExternal factorGraphExternal = new SlamWrapper.FactorGraphExternal();

      Pose3D poseInitial = new Pose3D();
      RigidBodyTransform odometry = new RigidBodyTransform();

      factorGraphExternal.setPoseInitialValue(1, PerceptionEuclidTools.toArray(poseInitial));
      factorGraphExternal.setPoseInitialValue(2, PerceptionEuclidTools.toArray(odometry));

      factorGraphExternal.optimize();

      factorGraphExternal.printResults();
   }

   @Test
   public void testOrientedPlaneFactors()
   {
      SlamWrapperNativeLibrary.load();

      SlamWrapper.FactorGraphExternal factorGraph = new SlamWrapper.FactorGraphExternal();

      Pose3D poseInitial = new Pose3D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      Pose3D odometry = new Pose3D(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      Vector4D planeOne = new Vector4D(0.0f, 0.0f, 2.0f, 0.0f);
      Vector4D planeMeasOne = new Vector4D(0.0f, 0.0f, 2.0f, 0.0f);
      Vector4D planeMeasTwo = new Vector4D(0.0f, 0.0f, 1.0f, 0.0f);

      factorGraph.addPriorPoseFactor(1,  PerceptionEuclidTools.toArray(poseInitial));
      factorGraph.addOdometryFactor(2, PerceptionEuclidTools.toArray(odometry));

      factorGraph.addOrientedPlaneFactor(1, 1, PerceptionEuclidTools.toArray(planeMeasOne));
      factorGraph.addOrientedPlaneFactor(1, 2, PerceptionEuclidTools.toArray(planeMeasTwo));

      factorGraph.setPoseInitialValue(1, PerceptionEuclidTools.toArray(poseInitial));
      factorGraph.setPoseInitialValue(2, PerceptionEuclidTools.toArray(odometry));
      factorGraph.setOrientedPlaneInitialValue(1, PerceptionEuclidTools.toArray(planeOne));

      factorGraph.optimize();

      factorGraph.printResults();


   }
}
