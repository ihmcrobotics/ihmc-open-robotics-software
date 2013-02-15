package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class ContactablePlaneBodyTools
{
   public static RectangularContactableBody createTypicalContactablePlaneBodyForTests(RigidBody rigidBody)
   {
      ReferenceFrame endEffectorFrame = rigidBody.getParentJoint().getFrameAfterJoint();
      
      Transform3D transform3D = new Transform3D();
      transform3D.setTranslation(new Vector3d(0.1, 0.2, -0.5));
      transform3D.rotY(Math.PI/2.0);
      
      ReferenceFrame soleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(rigidBody.getName() + "SoleFrame", endEffectorFrame, transform3D);
      
      double forward = 0.2;
      double back = -0.1;
      double left = 0.1;
      double right = -0.15;
      RectangularContactableBody rectangularContactableBody = new RectangularContactableBody(rigidBody, soleFrame, forward, back, left, right);
   
      return rectangularContactableBody;
   }
   
   public static RectangularContactableBody createRandomContactablePlaneBodyForTests(Random random, RigidBody rigidBody)
   {
      ReferenceFrame endEffectorFrame = rigidBody.getParentJoint().getFrameAfterJoint();
      
      Transform3D transform3D = RandomTools.generateRandomTransform(random);
      
      ReferenceFrame soleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(rigidBody.getName() + "SoleFrame", endEffectorFrame, transform3D);
      
      double forward = RandomTools.generateRandomDouble(random, 0.1, 0.2);
      double back = -RandomTools.generateRandomDouble(random, 0.1, 0.2);
      double left = RandomTools.generateRandomDouble(random, 0.1, 0.2);
      double right = - RandomTools.generateRandomDouble(random, 0.1, 0.2);
      RectangularContactableBody rectangularContactableBody = new RectangularContactableBody(rigidBody, soleFrame, forward, back, left, right);
   
      return rectangularContactableBody;
   }
}
