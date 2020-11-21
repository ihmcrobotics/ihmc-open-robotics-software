package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class CoMTrajectoryModelPredictiveControllerTest
{
   @Test
   public void testSimpleStanding()
   {
      double footLength = 0.22;
      double footWidth = 0.12;
      double gravityZ = -9.81;
      double dt = 0.001;
      double nominalHeight = 1.0;
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      CoMTrajectoryModelPredictiveController mpc = new CoMTrajectoryModelPredictiveController(gravityZ, nominalHeight, dt, testRegistry);

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartCopPosition(new FramePoint3D());
      contact.setEndCopPosition(new FramePoint3D());

      contactProviders.add(contact);

      mpc.solveForTrajectory(contactProviders);
      mpc.compute(0, 0.0);
   }

}
