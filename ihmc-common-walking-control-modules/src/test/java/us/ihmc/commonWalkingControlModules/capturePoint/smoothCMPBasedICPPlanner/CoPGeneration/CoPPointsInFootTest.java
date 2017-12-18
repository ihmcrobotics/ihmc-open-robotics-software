package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commons.Epsilons;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Test functionality of CoPPointsInFoot
 * @author ApoorvS
 *
 */
@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class CoPPointsInFootTest
{
   private static final String testClassName = "CoPPointsInFootTestRegistry";
   private static final double epsilon = Epsilons.ONE_BILLIONTH;
   private final double maxXInSoleFrame = -0.075;
   private final double minXInSoleFrame = -0.05;
   private final double maxYInSoleFrame = +0.05;
   private final double minYInSoleFrame = -0.075;
   private final double xToAnkle = 0.0;
   private final double yToAnkle = -0.15;
   private final double zToAnkle = 0.5;
   private final List<Point2D> footVertexList = Arrays.asList(new Point2D(maxXInSoleFrame, maxYInSoleFrame), new Point2D(maxXInSoleFrame, minYInSoleFrame),
                                                              new Point2D(minXInSoleFrame, maxYInSoleFrame), new Point2D(minXInSoleFrame, minYInSoleFrame));
   private final YoVariableRegistry registry = new YoVariableRegistry(testClassName);
   private final FootSpoof footSpoof = new FootSpoof("DummyFoot", xToAnkle, yToAnkle, zToAnkle, footVertexList, 0.5);
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame[] framesToRegister = {worldFrame, footSpoof.getSoleFrame()};
   private CoPPointsInFoot copPointsInFoot;

   @Before
   public void setup()
   {
      copPointsInFoot = new CoPPointsInFoot(testClassName, 0, framesToRegister, registry);
   }

   @After
   public void clean()
   {
      copPointsInFoot.reset();
      registry.clear();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAddCoPPointToList()
   {
      assertTrue(copPointsInFoot.isEmpty());
      copPointsInFoot.addWayPoint(CoPPointName.BALL_COP);
      copPointsInFoot.addWayPoint(CoPPointName.HEEL_COP);
      assertTrue(copPointsInFoot.getNumberOfCoPPoints() == 2);
      assertTrue(copPointsInFoot.getCoPPointList().get(0).checkCoPPointMatch(CoPPointName.BALL_COP));
      assertTrue(copPointsInFoot.getCoPPointList().get(1).checkCoPPointMatch(CoPPointName.HEEL_COP));
      copPointsInFoot.reset();
      assertTrue(copPointsInFoot.getCoPPointList().isEmpty());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAddandSetIncludingFrameWithFramePoint()
   {
      FramePoint3D testLocation = new FramePoint3D(footSpoof.getSoleFrame(), Math.random(), Math.random(), Math.random());
      assertTrue(copPointsInFoot.isEmpty());
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.BALL_COP, 0.2, testLocation);
      assertTrue(copPointsInFoot.getCoPPointList().size() == 1);

      assertTrue(copPointsInFoot.get(0).getPosition().epsilonEquals(testLocation, epsilon));
      assertTrue(MathTools.epsilonEquals(copPointsInFoot.get(0).getTime(), 0.2, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAddAndSetIncludingFrameWithYoFramePoint()
   {
      YoFramePoint testLocation1 = new YoFramePoint("TestLocation1", footSpoof.getSoleFrame(), null);
      YoFramePoint testLocation2 = new YoFramePoint("TestLocation2", footSpoof.getSoleFrame(), null);
      testLocation1.set(Math.random(), Math.random(), Math.random());
      testLocation2.set(Math.random(), Math.random(), Math.random());
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.HEEL_COP, 0.87, testLocation1);
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.BALL_COP, 0.12, testLocation2);
      assertTrue(copPointsInFoot.getCoPPointList().size() == 2);

      assertTrue(copPointsInFoot.get(0).getPosition().epsilonEquals(testLocation1.getFramePointCopy(), epsilon));
      assertTrue(MathTools.epsilonEquals(copPointsInFoot.get(0).getTime(), 0.87, epsilon));
      assertTrue(copPointsInFoot.get(1).getPosition().epsilonEquals(testLocation2.getFramePointCopy(), epsilon));
      assertTrue(MathTools.epsilonEquals(copPointsInFoot.get(1).getTime(), 0.12, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAddAndSetIncludingFrameWithCoPTrajectoryPoint()
   {
      CoPTrajectoryPoint testLocation1 = new CoPTrajectoryPoint("TestLocation1", "", null, framesToRegister);
      CoPTrajectoryPoint testLocation2 = new CoPTrajectoryPoint("TestLocation2", "", null, framesToRegister);
      testLocation1.changeFrame(footSpoof.getSoleFrame());
      testLocation1.setPosition(new FramePoint3D(footSpoof.getSoleFrame(), Math.random(), Math.random(), Math.random()));
      testLocation2.changeFrame(footSpoof.getSoleFrame());
      testLocation2.setPosition(new FramePoint3D(footSpoof.getSoleFrame(), Math.random(), Math.random(), Math.random()));
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.HEEL_COP, 0.87, testLocation1);
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.BALL_COP, 0.12, testLocation2);
      assertTrue(copPointsInFoot.getCoPPointList().size() == 2);

      assertTrue(copPointsInFoot.get(0).getPosition().epsilonEquals(testLocation1.getPosition().getFramePointCopy(), epsilon));
      assertTrue(MathTools.epsilonEquals(copPointsInFoot.get(0).getTime(), 0.87, epsilon));
      assertTrue(copPointsInFoot.get(1).getPosition().epsilonEquals(testLocation2.getPosition().getFramePointCopy(), epsilon));
      assertTrue(MathTools.epsilonEquals(copPointsInFoot.get(1).getTime(), 0.12, epsilon));
      FramePoint3D tempFramePointForTesting = new FramePoint3D(footSpoof.getSoleFrame());
      copPointsInFoot.getFinalCoPPosition(tempFramePointForTesting);
      assertTrue(tempFramePointForTesting.epsilonEquals(testLocation2.getPosition().getFramePointCopy(), epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetFeetLocations()
   {
      copPointsInFoot.setFeetLocation(new FramePoint3D(worldFrame, 0.2, 1.35, 2.1), new FramePoint3D(worldFrame, 1.3, 2.4, 6.6));
      FramePoint3D framePointForTesting = new FramePoint3D();
      copPointsInFoot.getSwingFootLocation(framePointForTesting);
      assertTrue(framePointForTesting.getReferenceFrame() == worldFrame);
      assertTrue(framePointForTesting.getX() == 0.2);
      assertTrue(framePointForTesting.getY() == 1.35);
      assertTrue(framePointForTesting.getZ() == 2.1);
      copPointsInFoot.getSupportFootLocation(framePointForTesting);
      assertTrue(framePointForTesting.getX() == 1.3);
      assertTrue(framePointForTesting.getY() == 2.4);
      assertTrue(framePointForTesting.getZ() == 6.6);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testChangeFrame()
   {
      copPointsInFoot.setFeetLocation(new FramePoint3D(worldFrame, 0.2, 0.1, 0.1), new FramePoint3D(worldFrame, 0.2, -0.1, 0.1));
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.BALL_COP, 0.2, new FramePoint3D(worldFrame, 0.2, 0.15, 0.1));
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.HEEL_COP, 0.8, new FramePoint3D(worldFrame, 0.15, -0.05, 0.11));
      copPointsInFoot.changeFrame(footSpoof.getSoleFrame());
      FramePoint3D tempFramePoint = new FramePoint3D();
      copPointsInFoot.getSupportFootLocation(tempFramePoint);
      assertTrue(tempFramePoint.getReferenceFrame() == footSpoof.getSoleFrame());
      assertTrue(tempFramePoint.getX() == 0.2 + xToAnkle);
      assertTrue(tempFramePoint.getY() == -0.1 + yToAnkle);
      assertTrue(tempFramePoint.getZ() == 0.1 + zToAnkle);
      assertTrue(copPointsInFoot.get(0).getReferenceFrame() == footSpoof.getSoleFrame());
      copPointsInFoot.get(0).getPosition(tempFramePoint);
      assertTrue(tempFramePoint.getX() == 0.2 + xToAnkle);
      assertTrue(tempFramePoint.getY() == 0.15 + yToAnkle);
      assertTrue(tempFramePoint.getZ() == 0.1 + zToAnkle);
      assertTrue(copPointsInFoot.get(1).getReferenceFrame() == footSpoof.getSoleFrame());
      copPointsInFoot.get(1).getPosition(tempFramePoint);
      assertTrue(tempFramePoint.getX() == 0.15 + xToAnkle);
      assertTrue(tempFramePoint.getY() == -0.05 + yToAnkle);
      assertTrue(tempFramePoint.getZ() == 0.11 + zToAnkle);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRegisterFrame()
   {
      double newFrameOriginX = 1;
      double newFrameOriginY = -1;
      double newFrameOriginZ = 1;

      copPointsInFoot.setFeetLocation(new FramePoint3D(worldFrame, 0.2, 0.1, 0.1), new FramePoint3D(worldFrame, 0.2, -0.1, 0.1));
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.BALL_COP, 0.2, new FramePoint3D(worldFrame, 0.2, 0.15, 0.1));
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.HEEL_COP, 0.8, new FramePoint3D(worldFrame, 0.15, -0.05, 0.11));
      ReferenceFrame newFrameToRegister = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("RandomFrameToRegister", worldFrame,
                                                                                                         new RigidBodyTransform(new Quaternion(),
                                                                                                                                new FrameVector3D(worldFrame,
                                                                                                                                                  newFrameOriginX,
                                                                                                                                                  newFrameOriginY,
                                                                                                                                                  newFrameOriginZ)));
      copPointsInFoot.registerReferenceFrame(newFrameToRegister);
      copPointsInFoot.changeFrame(newFrameToRegister);
      FramePoint3D tempFramePoint = new FramePoint3D();
      copPointsInFoot.getSupportFootLocation(tempFramePoint);
      assertTrue(tempFramePoint.getReferenceFrame() == newFrameToRegister);
      assertTrue(tempFramePoint.getX() == 0.2 + newFrameOriginX);
      assertTrue(tempFramePoint.getY() == -0.1 + newFrameOriginY);
      assertTrue(tempFramePoint.getZ() == 0.1 + newFrameOriginZ);
      assertTrue(copPointsInFoot.get(0).getReferenceFrame() == newFrameToRegister);
      copPointsInFoot.get(0).getPosition(tempFramePoint);
      assertTrue(tempFramePoint.getX() == 0.2 + newFrameOriginX);
      assertTrue(tempFramePoint.getY() == 0.15 + newFrameOriginY);
      assertTrue(tempFramePoint.getZ() == 0.1 + newFrameOriginZ);
      assertTrue(copPointsInFoot.get(1).getReferenceFrame() == newFrameToRegister);
      copPointsInFoot.get(1).getPosition(tempFramePoint);
      assertTrue(tempFramePoint.getX() == 0.15 + newFrameOriginX);
      assertTrue(tempFramePoint.getY() == -0.05 + newFrameOriginY);
      assertTrue(tempFramePoint.getZ() == 0.11 + newFrameOriginZ);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVisualization()
   {
      YoGraphicsList dummyGraphicsList = new YoGraphicsList("DummyGraphics");
      ArtifactList dummyArtifactList = new ArtifactList("DummyArtifacts");
      copPointsInFoot.setupVisualizers(dummyGraphicsList, dummyArtifactList, 0.05);
      assertTrue(dummyArtifactList.getArtifacts().size() == 10);
      assertTrue(dummyGraphicsList.getYoGraphics().size() == 10);
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.BALL_COP, 1.0, new FramePoint3D(footSpoof.getSoleFrame(), 1.0, 2.1, 3.1));
      assertTrue(dummyGraphicsList.getYoGraphics().get(0) instanceof YoGraphicPosition);
      assertTrue(((YoGraphicPosition)dummyGraphicsList.getYoGraphics().get(0)).getX() == 1.0 - xToAnkle);
      assertTrue(((YoGraphicPosition)dummyGraphicsList.getYoGraphics().get(0)).getY() == 2.1 - yToAnkle);
      assertTrue(((YoGraphicPosition)dummyGraphicsList.getYoGraphics().get(0)).getZ() == 3.1 - zToAnkle);
   }

}
