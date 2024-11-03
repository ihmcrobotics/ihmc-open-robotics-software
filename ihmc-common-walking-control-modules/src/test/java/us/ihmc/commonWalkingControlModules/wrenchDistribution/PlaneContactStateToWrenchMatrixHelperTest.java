package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class PlaneContactStateToWrenchMatrixHelperTest
{
   private static final double footLength = 0.2;
   private static final double footWidth = 0.1;

   @Test
   public void testComputeWrenchJacobian()
   {
      PoseReferenceFrame soleFrame = new PoseReferenceFrame("soleFrame", ReferenceFrame.getWorldFrame());
      RigidBodyBasics foot = new RigidBody("foot", soleFrame);

      Random random = new Random(1738L);

      int posesOfSoleFrameToTest = 100;
      int planePosesToTest = 100;


      ContactablePlaneBody contactablePlaneBody = createContactablePlaneBody(foot, soleFrame);
      PlaneContactStateToWrenchMatrixHelper helper = new PlaneContactStateToWrenchMatrixHelper(contactablePlaneBody,
                                                                                               ReferenceFrame.getWorldFrame(),
                                                                                               4,
                                                                                               4,
                                                                                               new ZeroConeRotationCalculator(),
                                                                                               new YoRegistry("test"));

      CoPObjectiveCalculator copObjectiveCalculator = new CoPObjectiveCalculator();

      for (int solePosesId = 0; solePosesId < posesOfSoleFrameToTest; solePosesId++)
      {
         soleFrame.setPoseAndUpdate(EuclidFrameRandomTools.nextFramePose3D(random, ReferenceFrame.getWorldFrame(), 10.0, 10.0));

         PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
         planeContactStateCommand.setContactingRigidBody(foot);
         planeContactStateCommand.setCoefficientOfFriction(1.0);
         planeContactStateCommand.setContactNormal(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0));
         planeContactStateCommand.setHasContactStateChanged(true);
         contactablePlaneBody.getContactPoints2d().forEach(planeContactStateCommand::addPointInContact);

         helper.setPlaneContactStateCommand(planeContactStateCommand);
         helper.computeMatrices(1e-8, 1e-9, new Vector2D(1e-5, 1e-5), new Vector2D(1e-6, 1e-6));

         DMatrixRMaj worldJacobian = new DMatrixRMaj(6, helper.getRhoSize());
         DMatrixRMaj planeJacobian = new DMatrixRMaj(6, helper.getRhoSize());
         helper.computeWrenchJacobianInFrame(ReferenceFrame.getWorldFrame(), worldJacobian);

         DMatrixRMaj worldWrenchMatrix = new DMatrixRMaj(6, 1);
         DMatrixRMaj planeWrenchMatrix = new DMatrixRMaj(6, 1);
         DMatrixRMaj randomRho = RandomMatrices_DDRM.rectangle(helper.getRhoSize(), 1, random);
         CommonOps_DDRM.mult(worldJacobian, randomRho, worldWrenchMatrix);

         Wrench worldWrench = new Wrench(foot.getBodyFixedFrame(), ReferenceFrame.getWorldFrame());
         worldWrench.set(worldWrenchMatrix);

         for (int planePosesId = 0; planePosesId < planePosesToTest; planePosesId++)
         {
            ReferenceFrame planeToTest = EuclidFrameRandomTools.nextReferenceFrame(random);

            helper.computeWrenchJacobianInFrame(planeToTest, planeJacobian);
            CommonOps_DDRM.mult(planeJacobian, randomRho, planeWrenchMatrix);

            Wrench planeWrench = new Wrench(foot.getBodyFixedFrame(), planeToTest);
            planeWrench.set(planeWrenchMatrix);

            planeWrench.changeFrame(ReferenceFrame.getWorldFrame());

            EuclidFrameTestTools.assertEquals(worldWrench.getAngularPart(), planeWrench.getAngularPart(), 1e-6);
            EuclidFrameTestTools.assertEquals(worldWrench.getLinearPart(), planeWrench.getLinearPart(), 1e-6);
         }
      }
   }

   private ContactablePlaneBody createContactablePlaneBody(RigidBodyBasics foot, ReferenceFrame soleFrame)
   {
      double halfLength = footLength / 2.0;
      double halfWidth = footWidth / 2.0;

      List<Point2D> points = new ArrayList<>();
      points.add(new Point2D(halfLength, halfWidth));
      points.add(new Point2D(halfLength, -halfWidth));
      points.add(new Point2D(-halfLength, -halfWidth));
      points.add(new Point2D(-halfLength, halfWidth));

      return new ListOfPointsContactablePlaneBody(foot, soleFrame, points);
   }

}
