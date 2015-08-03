package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.NonFlatGroundPlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.FootSpoof;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class SplineBasedCoMHeightTrajectoryGeneratorTest
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void raisedFlatSingleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeightAboveAnkle = 1.0;
      double contactAnkleZ = 1.5;
      Point3d supportFootAnklePosition = new Point3d(0.0, 0.0, contactAnkleZ);

      Point3d swingFootAnklePosition = new Point3d(1.0, 1.0, contactAnkleZ);
      boolean doubleSupport = false;

      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(0.5, 5.0, 1.0);
      Point3d point4 = new Point3d(1.0, 2.0, 0.0);
      Point3d point5 = new Point3d(1.5, -0.5, 5.0);

      Point3d[] coMQueries = new Point3d[] {point1, point2, point3, point4, point5};

      double expectedHeight = contactAnkleZ + nominalCoMHeightAboveAnkle;
      double[] expectedHeights = new double[] {expectedHeight, expectedHeight, expectedHeight, expectedHeight, expectedHeight};

      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeightAboveAnkle, supportFootAnklePosition, swingFootAnklePosition, doubleSupport,
              coMQueries, expectedHeights, 1e-7);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void raisedFlatDoubleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeightAboveAnkle = 1.0;
      double contactAnkleZ = 0.5;

      Point3d supportFootAnklePosition0 = new Point3d(0.0, 0.0, contactAnkleZ);
      Point3d supportFootAnklePosition1 = new Point3d(1.0, 0.0, contactAnkleZ);
      boolean doubleSupport = true;

      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(0.5, 5.0, 1.0);
      Point3d point4 = new Point3d(1.0, 2.0, 0.0);
      Point3d point5 = new Point3d(1.5, -0.5, 5.0);

      Point3d[] coMQueries = new Point3d[] {point1, point2, point3, point4, point5};

      double expectedHeight = contactAnkleZ + nominalCoMHeightAboveAnkle;
      double[] expectedHeights = new double[] {expectedHeight, expectedHeight, expectedHeight, expectedHeight, expectedHeight};

      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeightAboveAnkle, supportFootAnklePosition0, supportFootAnklePosition1, doubleSupport,
              coMQueries, expectedHeights, 1e-7);
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void raisedSlopedSingleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeightAboveSupportAnkle = 1.0;
      double contactAnkleZ = 0.5;
      double swingX = 1.31;
      double slope = 1.17;

      Point3d supportFootAnklePosition = new Point3d(0.0, 0.0, contactAnkleZ);
      Point3d swingFootAnklePosition = new Point3d(swingX, 0.0, contactAnkleZ + swingX * slope);
      boolean doubleSupport = false;

      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(swingX, 0.5, 0.0);
      Point3d point4 = new Point3d(swingX + 0.5, 2.0, 5.0);

      Point3d[] coMQueries = new Point3d[] {point1, point2, point3, point4};

      double expectedCoMHeightAtSupport = contactAnkleZ + nominalCoMHeightAboveSupportAnkle;
      double expectedCoMHeightAtSwing = contactAnkleZ + nominalCoMHeightAboveSupportAnkle + swingX * slope;

      double[] expectedHeights = new double[] {expectedCoMHeightAtSupport, expectedCoMHeightAtSupport, expectedCoMHeightAtSwing, expectedCoMHeightAtSwing};

      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeightAboveSupportAnkle, supportFootAnklePosition, swingFootAnklePosition,
              doubleSupport, coMQueries, expectedHeights, 1e-7);
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void raisedSlopedDoubleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeightAboveAnkle = 1.0;
      Point3d supportFootAnklePosition0 = new Point3d(0.0, 0.0, 0.5);
      Point3d supportFootAnklePosition1 = new Point3d(1.0, 1.0, 1.5);
      boolean doubleSupport = true;

      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(1.5, 0.5, 0.0);
      Point3d point4 = new Point3d(1.5, 2.0, 5.0);

      Point3d[] coMQueries = new Point3d[] {point1, point2, point3, point4};

      double[] expectedHeights = new double[] {1.5, 1.5, 2.5, 2.5};

      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeightAboveAnkle, supportFootAnklePosition0, supportFootAnklePosition1, doubleSupport,
              coMQueries, expectedHeights, 1e-7);
   }

   public void allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(double nominalCoMHeight, Point3d bodyFramePosition0, Point3d bodyFramePosition1,
           boolean doubleSupport, Point3d[] coMQueries, double[] expectedHeights, double epsilon)
   {
      double[][] expectedOutputs = new double[expectedHeights.length][6];
      for (int i = 0; i < expectedHeights.length; i++)
      {
         expectedOutputs[i][0] = expectedHeights[i];

         for (int j = 1; j < 6; j++)
         {
            expectedOutputs[i][j] = 0.0;
         }
      }

      generalCoMHeightTrajectoryTest(nominalCoMHeight, bodyFramePosition0, bodyFramePosition1, doubleSupport, coMQueries, expectedOutputs, epsilon);
   }

   public void generalCoMHeightTrajectoryTest(double nominalCoMHeight, Point3d contactFramePosition0, Point3d contactFramePosition1, boolean doubleSupport,
           Point3d[] coMQueries, double[][] expectedOutputs, double epsilon)
   {
      List<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
      Footstep nextFootstep = null;
      
      RigidBody rigidBody1 = new RigidBody("yop1", ReferenceFrame.getWorldFrame());
      contactStates.add(new NonFlatGroundPlaneContactState(0.2, 0.1, contactFramePosition0, new Vector3d(0.0, 0.0, 1.0), 1e-7, rigidBody1));

      if (doubleSupport)
      {
         // leave nextFootstep as null
         RigidBody rigidBody2 = new RigidBody("yop2", ReferenceFrame.getWorldFrame());
         contactStates.add(new NonFlatGroundPlaneContactState(0.2, 0.1, contactFramePosition1, new Vector3d(0.0, 0.0, 1.0), 1e-7, rigidBody2));
      }
      else
      {
         // do not add a second element to contactStates
         nextFootstep = getFootstep(contactFramePosition1);
      }

      CoMHeightPartialDerivativesData coMHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
      ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData = new ContactStatesAndUpcomingFootstepData();

      centerOfMassHeightInputData.set(null, null, null, nextFootstep, contactStates);

      CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator = new SplineBasedHeightTrajectoryGenerator(nominalCoMHeight, null, registry);
      centerOfMassHeightTrajectoryGenerator.initialize(null, null, nextFootstep, contactStates);

      FramePoint centerOfMassHeightPoint = new FramePoint(ReferenceFrame.getWorldFrame());

      for (int i = 0; i < coMQueries.length; i++)
      {
         centerOfMassHeightInputData.setCenterOfMassAndPelvisZUpFrames(createCenterOfMassFrame(coMQueries[i]), null);
         centerOfMassHeightTrajectoryGenerator.solve(coMHeightPartialDerivativesData, centerOfMassHeightInputData);

         coMHeightPartialDerivativesData.getCoMHeight(centerOfMassHeightPoint);
         assertEquals(expectedOutputs[i][0], centerOfMassHeightPoint.getZ(), epsilon);
         assertEquals(expectedOutputs[i][1], coMHeightPartialDerivativesData.getPartialDzDx(), epsilon);
         assertEquals(expectedOutputs[i][2], coMHeightPartialDerivativesData.getPartialDzDy(), epsilon);
         assertEquals(expectedOutputs[i][3], coMHeightPartialDerivativesData.getPartialD2zDx2(), epsilon);
         assertEquals(expectedOutputs[i][4], coMHeightPartialDerivativesData.getPartialD2zDy2(), epsilon);
         assertEquals(expectedOutputs[i][5], coMHeightPartialDerivativesData.getPartialD2zDxDy(), epsilon);
      }
   }

   private Footstep getFootstep(Point3d contactFrameCenter)
   {
      ContactablePlaneBody foot = new FootSpoof("foot", 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, 0.0);
      
      Footstep nextFootstep = new Footstep(foot.getRigidBody(), RobotSide.LEFT, foot.getSoleFrame());
      
      FramePoint position = new FramePoint(worldFrame, contactFrameCenter);
      FrameOrientation orientation = new FrameOrientation(worldFrame);
      FramePose footstepPose = new FramePose(position, orientation);
      nextFootstep.setPose(footstepPose );
      
      return nextFootstep;
   }

   private ReferenceFrame createCenterOfMassFrame(Point3d centerOfMassLocation)
   {
      TranslationReferenceFrame centerOfMassFrame = new TranslationReferenceFrame("centerOfMass", worldFrame);

      centerOfMassFrame.updateTranslation(new FrameVector(worldFrame, centerOfMassLocation));
      centerOfMassFrame.update();

      return centerOfMassFrame;
   }
}
