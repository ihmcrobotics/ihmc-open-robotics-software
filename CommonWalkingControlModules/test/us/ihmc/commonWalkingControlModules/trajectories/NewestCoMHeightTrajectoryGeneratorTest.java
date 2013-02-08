package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.NonFlatGroundPlaneContactState;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootSpoof;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.utilities.math.geometry.FrameOrientation2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.TranslationReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class NewestCoMHeightTrajectoryGeneratorTest
{
   private final YoVariableRegistry registry = new YoVariableRegistry("NewestCoMHeightTrajectoryGeneratorTest");
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   
   @Test
   public void flatDoubleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeight = 1.0;
      Point3d contactFramePosition0 = new Point3d(0.0, 0.0, 0.0);
      Point3d contactFramePosition1 = new Point3d(1.0, 0.0, 0.0);
      boolean doubleSupport = true;
      
      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(0.5, 5.0, 1.0);
      Point3d point4 = new Point3d(1.0, 2.0, 0.0);
      Point3d point5 = new Point3d(1.5, -0.5, 5.0);
      
      Point3d[] coMQueries = new Point3d[]{point1, point2, point3, point4, point5};
      
      double[] expectedHeights = new double[]{1.0, 1.0, 1.0, 1.0, 1.0};
      
      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeight, contactFramePosition0, contactFramePosition1, doubleSupport, coMQueries, expectedHeights, 1e-7);
   }
   
   @Test
   public void flatSingleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeight = 1.0;
      Point3d contactFramePosition0 = new Point3d(0.0, 0.0, 0.0);
      Point3d contactFramePosition1 = new Point3d(1.0, 0.0, 0.0);
      boolean doubleSupport = false;

      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(0.5, 5.0, 1.0);
      Point3d point4 = new Point3d(1.0, 2.0, 0.0);
      Point3d point5 = new Point3d(1.5, -0.5, 5.0);
      
      Point3d[] coMQueries = new Point3d[]{point1, point2, point3, point4, point5};
      
      double[] expectedHeights = new double[]{1.0, 1.0, 1.0, 1.0, 1.0};
      
      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeight, contactFramePosition0, contactFramePosition1, doubleSupport, coMQueries, expectedHeights, 1e-7);
   }
   
   @Test
   public void raisedFlatSingleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeight = 1.0;
      Point3d contactFramePosition0 = new Point3d(0.0, 0.0, 0.5);
      Point3d contactFramePosition1 = new Point3d(1.0, 1.0, 0.5);
      boolean doubleSupport = false;
      
      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(0.5, 5.0, 1.0);
      Point3d point4 = new Point3d(1.0, 2.0, 0.0);
      Point3d point5 = new Point3d(1.5, -0.5, 5.0);
      
      Point3d[] coMQueries = new Point3d[]{point1, point2, point3, point4, point5};
      
      double[] expectedHeights = new double[]{1.5, 1.5, 1.5, 1.5, 1.5};
      
      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeight, contactFramePosition0, contactFramePosition1, doubleSupport, coMQueries, expectedHeights, 1e-7);
   }
   
   @Test
   public void raisedFlatDoubleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeight = 1.0;
      Point3d contactFramePosition0 = new Point3d(0.0, 0.0, 0.5);
      Point3d contactFramePosition1 = new Point3d(1.0, 0.0, 0.5);
      boolean doubleSupport = false;

      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(0.5, 5.0, 1.0);
      Point3d point4 = new Point3d(1.0, 2.0, 0.0);
      Point3d point5 = new Point3d(1.5, -0.5, 5.0);
      
      Point3d[] coMQueries = new Point3d[]{point1, point2, point3, point4, point5};
      
      double[] expectedHeights = new double[]{1.5, 1.5, 1.5, 1.5, 1.5};
      
      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeight, contactFramePosition0, contactFramePosition1, doubleSupport, coMQueries, expectedHeights, 1e-7);
   }
   
   @Test
   public void raisedSlopedSingleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeight = 1.0;
      Point3d contactFramePosition0 = new Point3d(0.0, 0.0, 0.5);
      Point3d contactFramePosition1 = new Point3d(1.0, 0.0, 1.5);
      boolean doubleSupport = false;
      
      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(1.5, 0.5, 0.0);
      Point3d point4 = new Point3d(1.5, 2.0, 5.0);
      
      Point3d[] coMQueries = new Point3d[]{point1, point2, point3, point4};
      
      double[] expectedHeights = new double[]{1.5, 1.5, 2.5, 2.5};
      
      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeight, contactFramePosition0, contactFramePosition1, doubleSupport, coMQueries, expectedHeights, 1e-7);
   }
   
   @Test
   public void raisedSlopedDoubleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeight = 1.0;
      Point3d contactFramePosition0 = new Point3d(0.0, 0.0, 0.5);
      Point3d contactFramePosition1 = new Point3d(1.0, 1.0, 1.5);
      boolean doubleSupport = true;
      
      Point3d point1 = new Point3d(-0.5, -0.5, 0.0);
      Point3d point2 = new Point3d(0.0, 0.0, 1.0);
      Point3d point3 = new Point3d(1.5, 0.5, 0.0);
      Point3d point4 = new Point3d(1.5, 2.0, 5.0);
      
      Point3d[] coMQueries = new Point3d[]{point1, point2, point3, point4};
      
      double[] expectedHeights = new double[]{1.5, 1.5, 2.5, 2.5};
      
      allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(nominalCoMHeight, contactFramePosition0, contactFramePosition1, doubleSupport, coMQueries, expectedHeights, 1e-7);
   }
   
   public void allExpectedPartialDerivativesZeroCoMHeightTrajectoryTest(double nominalCoMHeight, Point3d contactFramePosition0, Point3d contactFramePosition1, boolean doubleSupport, Point3d[] coMQueries, double[] expectedHeights, double epsilon)
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
      generalCoMHeightTrajectoryTest(nominalCoMHeight, contactFramePosition0, contactFramePosition1, doubleSupport, coMQueries, expectedOutputs, epsilon);
   }
   
   public void generalCoMHeightTrajectoryTest(double nominalCoMHeight, Point3d contactFramePosition0, Point3d contactFramePosition1, boolean doubleSupport, Point3d[] coMQueries, double[][] expectedOutputs, double epsilon)
   {
      List<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
      Footstep nextFootstep = null;
      contactStates.add(new NonFlatGroundPlaneContactState(0.2, 0.1, contactFramePosition0, new Vector3d(0.0, 0.0, 1.0), 1e-7));
      
      if (doubleSupport)
      {
         //leave nextFootstep as null
         contactStates.add(new NonFlatGroundPlaneContactState(0.2, 0.1, contactFramePosition1, new Vector3d(0.0, 0.0, 1.0), 1e-7));
      }
      else {
         //do not add a second element to contactStates
         nextFootstep = getFootstep(contactFramePosition1);
      }
      
      CoMHeightPartialDerivativesData coMHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
      ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData = new ContactStatesAndUpcomingFootstepData();
      
      centerOfMassHeightInputData.set(null, null, nextFootstep, contactStates);
      
      CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator = new NewestCoMHeightTrajectoryGenerator(nominalCoMHeight, registry);
      centerOfMassHeightTrajectoryGenerator.initialize(null, nextFootstep, contactStates);
      
      for (int i = 0; i < coMQueries.length; i++)
      {
         centerOfMassHeightInputData.setCenterOfMassFrame(createCenterOfMassFrame(coMQueries[i]));
         centerOfMassHeightTrajectoryGenerator.solve(coMHeightPartialDerivativesData, centerOfMassHeightInputData);
         assertEquals(expectedOutputs[i][0], coMHeightPartialDerivativesData.getCoMHeight(), epsilon);
         assertEquals(expectedOutputs[i][1], coMHeightPartialDerivativesData.getPartialDzDx(), epsilon);
         assertEquals(expectedOutputs[i][2], coMHeightPartialDerivativesData.getPartialDzDy(), epsilon);
         assertEquals(expectedOutputs[i][3], coMHeightPartialDerivativesData.getPartialD2zDx2(), epsilon);
         assertEquals(expectedOutputs[i][4], coMHeightPartialDerivativesData.getPartialD2zDy2(), epsilon);
         assertEquals(expectedOutputs[i][5], coMHeightPartialDerivativesData.getPartialD2zDxDy(), epsilon);
      }
   }

   private Footstep getFootstep(Point3d contactFrameCenter)
   {
      Footstep nextFootstep;
      FrameOrientation2d footstepOrientation = new FrameOrientation2d(worldFrame);
      FramePose2d footstepPose = new FramePose2d(new FramePoint2d(worldFrame, contactFrameCenter.getX(), contactFrameCenter.getY()), footstepOrientation);
      ContactablePlaneBody foot = new FootSpoof("foot", 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, 0.0);
      //PATH TODO: remove negative sign from contactFrameCenter.getZ() once grey fixes generateFootstep()
      nextFootstep = FootstepUtils.generateFootstep(footstepPose, foot, -contactFrameCenter.getZ(), new Vector3d(0.0, 0.0, 1.0));
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
