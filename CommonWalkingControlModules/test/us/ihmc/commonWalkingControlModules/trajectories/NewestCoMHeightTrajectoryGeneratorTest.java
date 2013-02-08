package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FlatGroundPlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
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
      boolean doubleSupport = false;
      Point3d CoMQuery = new Point3d(0.5, 0.5, 0.5);
      double expectedCoMHeight = 1.0;
      double expectedDzdx = 0.0;
      double expectedDzdy = 0.0;
      double expectedDdzddx = 0.0;
      double expectedDdzddy = 0.0;
      double expectedDdzdxdy = 0.0;
      double[] expectedOutput = new double[]{expectedCoMHeight, expectedDzdx, expectedDzdy, expectedDdzddx, expectedDdzddy, expectedDdzdxdy};
      
      generalCoMHeightTrajectoryTest(nominalCoMHeight, contactFramePosition0, contactFramePosition1, doubleSupport, CoMQuery, expectedOutput, 1e-7);
   }
   
   public void generalCoMHeightTrajectoryTest(double nominalCoMHeight, Point3d contactCenter0, Point3d contactCenter1, boolean doubleSupport, Point3d CoMQuery, double[] expectedOutput, double epsilon)
   {
      ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(CoMQuery);
      List<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
      Footstep nextFootstep = null;
      contactStates.add(new FlatGroundPlaneContactState(0.2, 0.1, contactCenter0, 0.0));
      
      if (doubleSupport)
      {
         //leave nextFootstep as null
         contactStates.add(new FlatGroundPlaneContactState(0.2, 0.1, contactCenter1, 0.0));
      }
      else {
         //do not add a second element to contactStates
         nextFootstep = getFootstep(contactCenter1);
      }
      
      CoMHeightPartialDerivativesData coMHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
      ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData = new ContactStatesAndUpcomingFootstepData();
      
      centerOfMassHeightInputData.set(centerOfMassFrame, null, nextFootstep, contactStates);
      
      CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator = new NewestCoMHeightTrajectoryGenerator(nominalCoMHeight, registry);
      centerOfMassHeightTrajectoryGenerator.initialize(null, nextFootstep, contactStates);
      centerOfMassHeightTrajectoryGenerator.solve(coMHeightPartialDerivativesData, centerOfMassHeightInputData);
      
      assertEquals(expectedOutput[0], coMHeightPartialDerivativesData.getCoMHeight(), epsilon);
      assertEquals(expectedOutput[1], coMHeightPartialDerivativesData.getPartialDzDx(), epsilon);
      assertEquals(expectedOutput[2], coMHeightPartialDerivativesData.getPartialDzDy(), epsilon);
      assertEquals(expectedOutput[3], coMHeightPartialDerivativesData.getPartialD2zDx2(), epsilon);
      assertEquals(expectedOutput[4], coMHeightPartialDerivativesData.getPartialD2zDy2(), epsilon);
      assertEquals(expectedOutput[5], coMHeightPartialDerivativesData.getPartialD2zDxDy(), epsilon);
   }

   private Footstep getFootstep(Point3d contactFrameCenter)
   {
      Footstep nextFootstep;
      FrameOrientation2d footstepOrientation = new FrameOrientation2d(worldFrame);
      FramePose2d footstepPose = new FramePose2d(new FramePoint2d(worldFrame, contactFrameCenter.getX(), contactFrameCenter.getY()), footstepOrientation);
      ContactablePlaneBody foot = new FootSpoof("foot", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      nextFootstep = FootstepUtils.generateFootstep(footstepPose, foot, contactFrameCenter.getZ(), new Vector3d(0.0, 0.0, 1.0));
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
