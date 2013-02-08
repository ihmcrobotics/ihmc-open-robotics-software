package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FlatGroundPlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootSpoof;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.utilities.math.geometry.FrameOrientation2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.TranslationReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class NewestCoMHeightTrajectoryGeneratorTest
{
   private final YoVariableRegistry registry = new YoVariableRegistry("NewestCoMHeightTrajectoryGeneratorTest");
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   @Test
   public void basicDoubleSupportCoMHeightTrajectoryTest()
   {
      double nominalCoMHeight = 1.0;
      Point3d contactCenter0 = new Point3d(0.0, 0.0, 0.0);
      Point3d contactCenter1 = new Point3d(1.0, 0.0, 0.0);
      boolean doubleSupport = false;
      Point3d CoMQuery = new Point3d(0.5, 0.5, 0.5);
      double[] expectedOutput = new double[6];
      
      generalCoMHeightTrajectoryTest(nominalCoMHeight, contactCenter0, contactCenter1, doubleSupport, CoMQuery, expectedOutput);
   }
   
   public void generalCoMHeightTrajectoryTest(double nominalCoMHeight, Point3d contactCenter0, Point3d contactCenter1, boolean doubleSupport, Point3d CoMQuery, double[] expectedOutput)
   {
      ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(CoMQuery);
      List<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
      Footstep nextFootstep = null;
      contactStates.add(new FlatGroundPlaneContactState(0.2, 0.1, contactCenter0));
      
      if (doubleSupport)
      {
         //leave nextFootstep as null
         contactStates.add(new FlatGroundPlaneContactState(0.2, 0.1, contactCenter1));
      }
      else {
         //do not add a second element to contactStates
         FrameOrientation2d footstepOrientation = new FrameOrientation2d(worldFrame);
         FramePose2d footstepPose = new FramePose2d(new FramePoint2d(worldFrame, contactCenter1.getX(), contactCenter1.getY()), footstepOrientation);
         RigidBody footBody = new RigidBody("footBody", worldFrame);
         ContactablePlaneBody foot = new FootSpoof("foot", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         nextFootstep = FootstepUtils.generateFootstep(footstepPose, foot, contactCenter1.getZ(), new Vector3d(0.0, 0.0, 1.0));
      }
      
      CenterOfMassHeightPartialDerivativesData coMHeightPartialDerivativesData = new CenterOfMassHeightPartialDerivativesData();
      CenterOfMassHeightInputData centerOfMassHeightInputData = new CenterOfMassHeightInputData();
      
      centerOfMassHeightInputData.set(centerOfMassFrame, null, nextFootstep, contactStates);
      
      CenterOfMassHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator = new NewestCoMHeightTrajectoryGenerator(nominalCoMHeight, registry);
      centerOfMassHeightTrajectoryGenerator.initialize(null, nextFootstep, contactStates);
      centerOfMassHeightTrajectoryGenerator.solve(coMHeightPartialDerivativesData, centerOfMassHeightInputData);
   }
   
   private ReferenceFrame createCenterOfMassFrame(Point3d centerOfMassLocation)
   {
      TranslationReferenceFrame centerOfMassFrame = new TranslationReferenceFrame("centerOfMass", worldFrame);
      
      centerOfMassFrame.updateTranslation(new FrameVector(worldFrame, centerOfMassLocation));
      centerOfMassFrame.update();
      
      return centerOfMassFrame;
   }
}
