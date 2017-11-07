package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce.FlatGroundContactForceOptimizer;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FlatGroundContactForceOptimizerTest
{

   private static final boolean showSCS = false;

   @Test(timeout = 30000)
   public void simpleTest()
   {
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");

      double friction = 0.8;
      int vectorsPerContact = 3;
      double regWeight = 1.0e-6;
      FlatGroundContactForceOptimizer optimizer = new FlatGroundContactForceOptimizer(friction, vectorsPerContact, regWeight, graphicsListRegistry, registry);

      List<FramePoint3D> contactPoints = new ArrayList<>();
      contactPoints.add(new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.5, -0.5, 0.0));
      contactPoints.add(new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.5, 0.5, 0.0));
      contactPoints.add(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5, 0.5, 0.0));
      contactPoints.add(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5, -0.5, 0.0));

      Point3D comPosition = new Point3D(0.0, 0.0, 0.6);


      SimulationConstructionSet scs = null;

      if (showSCS)
      {
         scs = new SimulationConstructionSet(new Robot(getClass().getSimpleName()));
         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.addYoVariableRegistry(registry);
      }

      FrameVector3D torque = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.3, 0.0, 0.2);
      WeightMatrix6D weights = new WeightMatrix6D();
      weights.setAngularWeights(1.0, 1.0, 1.0);
      weights.setLinearWeights(1.0, 1.0, 1.0);

      for (int i = 0; i < 100; i++)
      {
         double x = Math.sin(Math.PI * 2.0 * i / 100.0);
         double y = Math.cos(Math.PI * 2.0 * i / 100.0);
         double z = 2.0 * (Math.sin(Math.PI * 2.0 * i / 25.0) + 1.0);
         Vector3D forceVector = new Vector3D(x, y, z);
         forceVector.scale(0.2);

         FramePoint3D centerOfMass = new FramePoint3D(ReferenceFrame.getWorldFrame(), comPosition);
         FrameVector3D force = new FrameVector3D(ReferenceFrame.getWorldFrame(), forceVector);

         assertTrue(optimizer.compute(contactPoints, centerOfMass, force, torque, weights));

         if (showSCS)
         {
            scs.setTime(i);
            scs.tickAndUpdate();
         }
      }

      if (showSCS)
      {
         scs.setOutPoint();
         scs.cropBuffer();
         scs.setCameraTrackingOffsets(comPosition.getX(), comPosition.getY(), comPosition.getZ() / 2.0);
         scs.setCameraTracking(true, true, true, true);
         scs.setPlaybackRealTimeRate(0.01);
         scs.play();
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
   }
}
