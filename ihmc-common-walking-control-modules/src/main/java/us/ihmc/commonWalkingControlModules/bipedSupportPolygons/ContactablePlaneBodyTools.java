package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class ContactablePlaneBodyTools
{

   public static RectangularContactableBody createTypicalContactablePlaneBodyForTests(RigidBodyBasics rigidBody, ReferenceFrame endEffectorFrame)
   {
      RigidBodyTransform transform3D = new RigidBodyTransform();
      transform3D.getTranslation().set(0.1, 0.2, -0.5);
      transform3D.setRotationPitchAndZeroTranslation(Math.PI / 2.0);

      ReferenceFrame soleFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(rigidBody.getName() + "SoleFrame", endEffectorFrame, transform3D);

      double forward = 0.2;
      double back = -0.1;
      double left = 0.1;
      double right = -0.15;
      RectangularContactableBody rectangularContactableBody = new RectangularContactableBody(rigidBody, soleFrame, forward, back, left, right);

      return rectangularContactableBody;
   }

   public static RectangularContactableBody createRandomContactablePlaneBodyForTests(Random random, RigidBodyBasics rigidBody)
   {
      //TODO: Use a better frame than just world here.
      ReferenceFrame endEffectorFrame = ReferenceFrame.getWorldFrame(); //rigidBody.getParentJoint().getFrameAfterJoint();

      RigidBodyTransform transform3D = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      ReferenceFrame soleFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(rigidBody.getName() + "SoleFrame", endEffectorFrame, transform3D);

      double forward = 0.2;
      double back = -0.1;
      double left = 0.1;
      double right = -0.15;
      RectangularContactableBody rectangularContactableBody = new RectangularContactableBody(rigidBody, soleFrame, forward, back, left, right);

      return rectangularContactableBody;
   }
}
