package us.ihmc.avatar.footstepSnapper;

import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.footstepPlanning.PlanarRegionEndToEndConversionTest;
import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepSnapper;
import us.ihmc.avatar.stepAdjustment.SimpleSteppableRegionsCalculator;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingEnvironmentalConstraintParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTestTools;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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
                                                                            new SteppingEnvironmentalConstraintParameters(),
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

      FootstepDataMessage dataMessage = new FootstepDataMessage();
      snapper.adjustFootstep(new FramePose3D(), poseOnGround, RobotSide.LEFT, dataMessage);
      snappedPoseOnGround.getPosition().set(dataMessage.getLocation());
      snappedPoseOnGround.getOrientation().set(dataMessage.getOrientation());
      EuclidFrameTestTools.assertEquals(expectedPoseOnGround, snappedPoseOnGround, 1e-5);

      snapper.adjustFootstep(new FramePose3D(), poseOnBlock, RobotSide.LEFT, dataMessage);
      snappedPoseOnBlock.getPosition().set(dataMessage.getLocation());
      snappedPoseOnBlock.getOrientation().set(dataMessage.getOrientation());
      EuclidFrameTestTools.assertEquals(expectedPoseOnBlock, snappedPoseOnBlock, 1e-5);

      snapper.adjustFootstep(new FramePose3D(), poseOnBlockEdge, RobotSide.LEFT, dataMessage);
      snappedPoseOnBlockEdge.getPosition().set(dataMessage.getLocation());
      snappedPoseOnBlockEdge.getOrientation().set(dataMessage.getOrientation());
      EuclidFrameTestTools.assertEquals(expectedPoseOnBlockEdge, snappedPoseOnBlockEdge, 1e-5);
   }

   @Test
   public void testSteppingBackwardsOffTheWorld()
   {
      ConvexPolygon2D footPolygon = createFootPolygonForTest();

      SimpleSteppableRegionsCalculator steppableRegionsCalculator = new SimpleSteppableRegionsCalculator();
      PlanarRegionFootstepSnapper snapper = new PlanarRegionFootstepSnapper(new SideDependentList<>(footPolygon, footPolygon),
                                                                            steppableRegionsCalculator,
                                                                            new SteppingEnvironmentalConstraintParameters(),
                                                                            new YoRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(0.05, 0.05);
      groundPolygon.addVertex(0.05, -0.05);
      groundPolygon.addVertex(0.4, 0.3);
      groundPolygon.addVertex(0.4, -0.3);
      groundPolygon.update();

      PlanarRegionCommand groundRegion = new PlanarRegionCommand();
      groundRegion.setRegionProperties(0, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      groundRegion.getConvexPolygons().add().set(groundPolygon);
      groundPolygon.getVertexBufferView().forEach(point ->
                                                  {
                                                     groundRegion.getConcaveHullsVertices().add().set(point);
                                                  });



      PlanarRegionsListCommand listCommand = new PlanarRegionsListCommand();
      listCommand.addPlanarRegionCommand(groundRegion);

      steppableRegionsCalculator.consume(listCommand);

      FramePose3D stanceFootPose = new FramePose3D();
      stanceFootPose.setY(-0.1);
      stanceFootPose.setZ(-0.1);

      FramePose2D inPlacePose = new FramePose2D();
      FramePose2D yawingInPlacePose = new FramePose2D();
      FramePose2D steppingBackAndYawingPose = new FramePose2D();

      inPlacePose.set(0.0, 0.1, 0.0);
      yawingInPlacePose.set(0.0, 0.1, 0.3);
      steppingBackAndYawingPose.set(-0.3, 0.1, -0.3);

      FramePose3D snappedPoseInPlace = new FramePose3D();
      FramePose3D snappedPoseYawingInPlace = new FramePose3D();
      FramePose3D snappedPoseBackAndYawing = new FramePose3D();

      FramePose3D expectedPoseInPlace = new FramePose3D();
      FramePose3D expectedPoseYawingInPlace = new FramePose3D();
      FramePose3D expectedPoseBackAndYawing = new FramePose3D();

      expectedPoseInPlace.set(inPlacePose);
      expectedPoseInPlace.setZ(stanceFootPose.getZ());
      expectedPoseYawingInPlace.set(yawingInPlacePose);
      expectedPoseYawingInPlace.setZ(stanceFootPose.getZ());
      expectedPoseBackAndYawing.set(steppingBackAndYawingPose);
      expectedPoseBackAndYawing.setZ(stanceFootPose.getZ());

      FootstepDataMessage dataMessage = new FootstepDataMessage();

      snapper.adjustFootstep(stanceFootPose, inPlacePose, RobotSide.LEFT, dataMessage);
      snappedPoseInPlace.getPosition().set(dataMessage.getLocation());
      snappedPoseInPlace.getOrientation().set(dataMessage.getOrientation());
      EuclidFrameTestTools.assertEquals(expectedPoseInPlace, snappedPoseInPlace, 1e-5);

      snapper.adjustFootstep(stanceFootPose, yawingInPlacePose, RobotSide.LEFT, dataMessage);
      snappedPoseYawingInPlace.getPosition().set(dataMessage.getLocation());
      snappedPoseYawingInPlace.getOrientation().set(dataMessage.getOrientation());
      EuclidFrameTestTools.assertEquals(expectedPoseYawingInPlace, snappedPoseYawingInPlace, 1e-5);

      snapper.adjustFootstep(stanceFootPose, steppingBackAndYawingPose, RobotSide.LEFT, dataMessage);
      snappedPoseBackAndYawing.getPosition().set(dataMessage.getLocation());
      snappedPoseBackAndYawing.getOrientation().set(dataMessage.getOrientation());

      EuclidFrameTestTools.assertEquals(expectedPoseBackAndYawing, snappedPoseBackAndYawing, 1e-5);
   }

   @Test
   public void testPossiblyBadProjection()
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(-0.108, 0.048);
      footPolygon.addVertex(0.108, 0.03);
      footPolygon.addVertex(0.108, -0.03);
      footPolygon.addVertex(-0.108, -0.048);
      footPolygon.update();

      SimpleSteppableRegionsCalculator steppableRegionsCalculator = new SimpleSteppableRegionsCalculator();
      PlanarRegionFootstepSnapper snapper = new PlanarRegionFootstepSnapper(new SideDependentList<>(footPolygon, footPolygon),
                                                                            steppableRegionsCalculator,
                                                                            new SteppingEnvironmentalConstraintParameters(),
                                                                            new YoRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(-0.33, -.04);
      groundPolygon.addVertex(0.413, 0.064);
      groundPolygon.addVertex(0.064, 0.038);
      groundPolygon.addVertex(0.264, 0.106);
      groundPolygon.addVertex(0.315, 0.030);
      groundPolygon.addVertex(0.244, -0.151);
      groundPolygon.addVertex(0.088, -0.322);
      groundPolygon.addVertex(0.036, -0.349);
      groundPolygon.addVertex(-0.078, -0.294);
      groundPolygon.addVertex(-0.267, -0.113);
      groundPolygon.update();

      Pose3D groundPose = new Pose3D(new Point3D(0.737, 0.005, 0.023), new Quaternion(0.003, -0.008, 0.0, 1.0));

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(-0.106, 0.005);
      blockPolygon.addVertex(-0.064, 0.051);
      blockPolygon.addVertex(0.021, 0.131);
      blockPolygon.addVertex(0.077, 0.173);
      blockPolygon.addVertex(0.089, -0.141);
      blockPolygon.addVertex(-0.037, -0.13);
      blockPolygon.addVertex(-0.082, -0.072);
      blockPolygon.update();

      Pose3D blockpose = new Pose3D(new Point3D(1.135, -0.723, 0.103), new Quaternion(0.009, -0.012, 0.0, 1.0));

      PlanarRegionCommand groundRegion = new PlanarRegionCommand();
      groundRegion.setRegionProperties(0, new RigidBodyTransform(groundPose));
      groundRegion.getConvexPolygons().add().set(groundPolygon);
      groundPolygon.getVertexBufferView().forEach(point ->
                                                  {
                                                     groundRegion.getConcaveHullsVertices().add().set(point);
                                                  });

      PlanarRegionCommand blockRegion = new PlanarRegionCommand();
      blockRegion.setRegionProperties(1, new RigidBodyTransform(blockpose));
      blockRegion.getConvexPolygons().add().set(blockPolygon);
      blockPolygon.getVertexBufferView().forEach(point ->
                                                 {
                                                    blockRegion.getConcaveHullsVertices().add().set(point);
                                                 });



      PlanarRegionsListCommand environmentList = new PlanarRegionsListCommand();
      environmentList.addPlanarRegionCommand(groundRegion);
      environmentList.addPlanarRegionCommand(blockRegion);

      steppableRegionsCalculator.consume(environmentList);

      Random random = new Random(1738L);

      for (int i = 0; i < 5; i++)
      {
         for (int rando = 0; rando < 50; rando++)
            steppableRegionsCalculator.consume(generateRandomRegions(random));

         steppableRegionsCalculator.consume(environmentList);

         FramePose3D stanceFootPose = new FramePose3D();
         stanceFootPose.setY(-0.1);
         stanceFootPose.setZ(0.022);

         FramePose3D step1Pose3D = new FramePose3D();
         step1Pose3D.getPosition().set(0.257, -0.168, 0.022);
         step1Pose3D.getOrientation().set(0.0, 0.0, -0.048, 0.999);

         FramePose3D step2Pose3D = new FramePose3D();
         step2Pose3D.getPosition().set(0.281, 0.081, 0.022);
         step2Pose3D.getOrientation().set(0.0, 0.0, -0.048, 0.999);

         FramePose3D step3Pose3D = new FramePose3D();
         step3Pose3D.getPosition().set(0.257, -0.168, 0.022);
         step3Pose3D.getOrientation().set(0.0, 0.0, -0.048, 0.999);

         FramePose2D step1Pose = new FramePose2D(step1Pose3D);
         FramePose2D step2Pose = new FramePose2D(step2Pose3D);
         FramePose2D step3Pose = new FramePose2D(step3Pose3D);

         FramePose3D snapped1Pose = new FramePose3D();
         FramePose3D snapped2Pose = new FramePose3D();
         FramePose3D snapped3Pose = new FramePose3D();

         FramePose3D expectedPose1 = new FramePose3D();
         FramePose3D expectedPose2 = new FramePose3D();
         FramePose3D expectedPose3 = new FramePose3D();

         expectedPose1.set(step1Pose);
         expectedPose1.setZ(stanceFootPose.getZ());
         expectedPose2.set(step2Pose3D);
         expectedPose2.setZ(stanceFootPose.getZ());
         expectedPose3.set(step3Pose3D);
         expectedPose3.setZ(stanceFootPose.getZ());

         FootstepDataMessage dataMessage = new FootstepDataMessage();

         snapper.adjustFootstep(stanceFootPose, step1Pose, RobotSide.LEFT, dataMessage);
         snapped1Pose.getPosition().set(dataMessage.getLocation());
         snapped1Pose.getOrientation().set(dataMessage.getOrientation());
         EuclidFrameTestTools.assertEquals(expectedPose1, snapped1Pose, 1e-5);

         snapper.adjustFootstep(stanceFootPose, step2Pose, RobotSide.LEFT, dataMessage);
         snapped2Pose.getPosition().set(dataMessage.getLocation());
         snapped2Pose.getOrientation().set(dataMessage.getOrientation());
         EuclidFrameTestTools.assertEquals(expectedPose2, snapped2Pose, 1e-5);

         snapper.adjustFootstep(stanceFootPose, step3Pose, RobotSide.LEFT, dataMessage);
         snapped3Pose.getPosition().set(dataMessage.getLocation());
         snapped3Pose.getOrientation().set(dataMessage.getOrientation());

         EuclidFrameTestTools.assertEquals(expectedPose3, snapped3Pose, 1e-5);
      }
   }


   @Test
   public void testSimpleBlockWithOutlierRegions()
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
                                                                            new SteppingEnvironmentalConstraintParameters(),
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

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      PlanarRegion groundRegion = new PlanarRegion(groundTransform, groundPolygon);

      RigidBodyTransform blockTransform = new RigidBodyTransform();
      blockTransform.getTranslation().set(0.3, 0.4, 0.25);
      PlanarRegion blockRegion = new PlanarRegion(blockTransform, blockPolygon);

      List<DataSet> allDatasets = DataSetIOTools.loadDataSets(PlanarRegionEndToEndConversionTest.buildFilter(PlanarRegionEndToEndConversionTest.getTestableFilter()));
      int iters = 20;
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         steppableRegionsCalculator.consume(getPlanarRegionCommandWithOutlierData(random, allDatasets, groundRegion, blockRegion));

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

         String failureMessage = "Failed on iteration " + iter;

         FootstepDataMessage dataMessage = new FootstepDataMessage();

         snapper.adjustFootstep(new FramePose3D(), poseOnGround, RobotSide.LEFT, dataMessage);
         snappedPoseOnGround.getPosition().set(dataMessage.getLocation());
         snappedPoseOnGround.getOrientation().set(dataMessage.getOrientation());
         EuclidFrameTestTools.assertEquals(failureMessage, expectedPoseOnGround, snappedPoseOnGround, 1e-5);

         snapper.adjustFootstep(new FramePose3D(), poseOnBlock, RobotSide.LEFT, dataMessage);
         snappedPoseOnBlock.getPosition().set(dataMessage.getLocation());
         snappedPoseOnBlock.getOrientation().set(dataMessage.getOrientation());
         EuclidFrameTestTools.assertEquals(failureMessage, expectedPoseOnBlock, snappedPoseOnBlock, 1e-5);

         snapper.adjustFootstep(new FramePose3D(), poseOnBlockEdge, RobotSide.LEFT, dataMessage);
         snappedPoseOnBlockEdge.getPosition().set(dataMessage.getLocation());
         snappedPoseOnBlockEdge.getOrientation().set(dataMessage.getOrientation());
         EuclidFrameTestTools.assertEquals(failureMessage, expectedPoseOnBlockEdge, snappedPoseOnBlockEdge, 1e-5);
      }
   }

   private static PlanarRegionsListCommand getPlanarRegionCommandWithOutlierData(Random random, List<DataSet> allDatasetsForOutliers, PlanarRegion... realRegions)
   {
      PlanarRegionsList planarRegionsForNoise = allDatasetsForOutliers.get(RandomNumbers.nextInt(random, 0, allDatasetsForOutliers.size() - 1)).getPlanarRegionsList();
      RigidBodyTransform transformForNoise = new RigidBodyTransform();
      transformForNoise.getTranslation().set(100.0, 100.0, -20.0);
      ArrayList<PlanarRegion> regionsToSend = new ArrayList<>();
      planarRegionsForNoise.getPlanarRegionsAsList().forEach(region ->
                                                             {
                                                                PlanarRegion regionCopy = region.copy();
                                                                regionCopy.applyTransform(transformForNoise);
                                                                regionsToSend.add(regionCopy);
                                                             });

      for (PlanarRegion realRegion : realRegions)
      {
         int indexToInsert = RandomNumbers.nextInt(random, 0, planarRegionsForNoise.getNumberOfPlanarRegions() - 1);
         regionsToSend.add(indexToInsert, realRegion);
      }

      PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(new PlanarRegionsList(regionsToSend));
      PlanarRegionsListCommand listCommand = new PlanarRegionsListCommand();
      listCommand.setFromMessage(message);

      return listCommand;
   }


   private static ConvexPolygon2D createFootPolygonForTest()
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(0.1, 0.05);
      footPolygon.addVertex(0.1, -0.05);
      footPolygon.addVertex(-0.1, 0.05);
      footPolygon.addVertex(-0.1, -0.05);
      footPolygon.update();

      return footPolygon;
   }

   private static PlanarRegionsListCommand generateRandomRegions(Random random)
   {
      PlanarRegionsListCommand listCommand = new PlanarRegionsListCommand();

      int regionsToGenerate = RandomNumbers.nextInt(random, 1, 20);
      for (int  i = 0; i < regionsToGenerate; i++)
      {
         ConvexPolygon2D randomRegion = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 1.0, 40);
         Pose3D regionPose = EuclidGeometryRandomTools.nextPose3D(random);

         PlanarRegionCommand region = new PlanarRegionCommand();
         region.setRegionProperties(i, new RigidBodyTransform(regionPose));
         region.getConvexPolygons().add().set(randomRegion);
         randomRegion.getVertexBufferView().forEach(point ->
                                                     {
                                                        region.getConcaveHullsVertices().add().set(point);
                                                     });

         listCommand.addPlanarRegionCommand(region);
      }

      return listCommand;
   }
}
