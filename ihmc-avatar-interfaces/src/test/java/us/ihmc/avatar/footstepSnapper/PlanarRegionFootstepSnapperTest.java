package us.ihmc.avatar.footstepSnapper;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepSnapper;
import us.ihmc.avatar.stepAdjustment.SimpleSteppableRegionsCalculator;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PlanarRegionFootstepSnapperTest
{
   @Test
   public void testSimpleBlock()
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(0.1, 0.05);
      footPolygon.addVertex(0.1, -0.05);
      footPolygon.addVertex(-0.1, 0.05);
      footPolygon.addVertex(-0.1, -0.05);
      footPolygon.update();

      SimpleSteppableRegionsCalculator steppableRegionsCalculator = new SimpleSteppableRegionsCalculator();
      PlanarRegionFootstepSnapper snapper = new PlanarRegionFootstepSnapper(new SideDependentList<>(footPolygon, footPolygon),
                                                                            steppableRegionsCalculator,
                                                                            new ConstraintOptimizerParameters(),
                                                                            new YoRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.update();

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(0.15, 0.15);
      blockPolygon.addVertex(0.15, -0.15);
      blockPolygon.addVertex(-0.15, -0.15);
      blockPolygon.addVertex(-0.15, 0.15);
      blockPolygon.update();

      PlanarRegionCommand groundRegion = new PlanarRegionCommand();
      groundRegion.setRegionProperties(0, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      groundRegion.getConvexPolygons().add().set(groundPolygon);
      groundPolygon.getVertexBufferView().forEach(point ->
            {
                  groundRegion.getConcaveHullsVertices().add().set(point);
            });

      PlanarRegionCommand blockRegion = new PlanarRegionCommand();
      blockRegion.setRegionProperties(1, new Point3D(0.3, 0.4, 0.25), new Vector3D(0.0, 0.0, 1.0));
      blockRegion.getConvexPolygons().add().set(blockPolygon);
      blockPolygon.getVertexBufferView().forEach(point ->
                                                 {
                                                    blockRegion.getConcaveHullsVertices().add().set(point);
                                                 });

      PlanarRegionsListCommand listCommand = new PlanarRegionsListCommand();
      listCommand.addPlanarRegionCommand(groundRegion);
      listCommand.addPlanarRegionCommand(blockRegion);

      steppableRegionsCalculator.consume(listCommand);

      FramePose2D poseOnGround = new FramePose2D();
      FramePose2D poseOnBlockEdge = new FramePose2D();
      FramePose2D poseOnBlock = new FramePose2D();

      poseOnGround.set(-0.15, 0.0, 0.0);
      poseOnBlock.set(0.3, 0.4, 0.0);
      poseOnBlockEdge.set(0.2, 0.25, 0.0);

      FramePose3D snappedPoseOnGround = new FramePose3D();
      FramePose3D snappedPoseOnBlockEdge = new FramePose3D();
      FramePose3D snappedPoseOnBlock = new FramePose3D();

      FramePose3D expectedPoseOnGround = new FramePose3D();
      FramePose3D expectedPoseOnBlockEdge = new FramePose3D();
      FramePose3D expectedPoseOnBlock = new FramePose3D();

      expectedPoseOnGround.set(new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.15, 0.0, 0.0), new FrameQuaternion());
      expectedPoseOnBlock.set(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.3, 0.4, 0.25), new FrameQuaternion());
      expectedPoseOnBlockEdge.set(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.3, 0.25), new FrameQuaternion());

      snapper.adjustFootstep(new FramePose3D(), poseOnGround, RobotSide.LEFT, snappedPoseOnGround);
      EuclidFrameTestTools.assertEquals(expectedPoseOnGround, snappedPoseOnGround, 1e-5);

      snapper.adjustFootstep(new FramePose3D(), poseOnBlock, RobotSide.LEFT, snappedPoseOnBlock);
      EuclidFrameTestTools.assertEquals(expectedPoseOnBlock, snappedPoseOnBlock, 1e-5);

      snapper.adjustFootstep(new FramePose3D(), poseOnBlockEdge, RobotSide.LEFT, snappedPoseOnBlockEdge);
      EuclidFrameTestTools.assertEquals(expectedPoseOnBlockEdge, snappedPoseOnBlockEdge, 1e-5);
   }
}
