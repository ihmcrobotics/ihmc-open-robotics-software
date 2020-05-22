package us.ihmc.robotics.physics;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

class CollidableTest
{

   @Test
   void testBugCollisionDetection()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("framA",
                                                                                                  worldFrame,
                                                                                                  new RigidBodyTransform(0.9659443261832176,
                                                                                                                         -3.5062314832876345E-9,
                                                                                                                         0.25874999268492704,
                                                                                                                         5.112545852281797E-5,
                                                                                                                         4.243717872272803E-9,
                                                                                                                         1.0,
                                                                                                                         -2.29164728162382E-9,
                                                                                                                         2.2876089263315645E-10,
                                                                                                                         -0.25874999268492704,
                                                                                                                         3.3116656577052046E-9,
                                                                                                                         0.9659443261832176,
                                                                                                                         0.15528951964594329));
      frameA.update();

      FrameBox3D shapeA = new FrameBox3D(frameA, 0.3, 0.3, 0.3);
      FrameBox3D shapeB = new FrameBox3D(worldFrame,
                                         new RigidBodyTransform(0.9659258262890683,
                                                                0.0,
                                                                0.25881904510252074,
                                                                -0.06470476127563018,
                                                                0.0,
                                                                1.0,
                                                                0.0,
                                                                0.0,
                                                                -0.25881904510252074,
                                                                0.0,
                                                                0.9659258262890683,
                                                                -0.24148145657226708),
                                         10000.000,
                                         10000.000,
                                         0.500);

      EuclidFrameShape3DCollisionResult collisionData = new EuclidFrameShape3DCollisionResult();
      PhysicsEngineTools.evaluateShape3DShape3DCollision(shapeA, shapeB, collisionData);

      FramePoint3D pointOnA = collisionData.getPointOnA();
      FramePoint3D pointOnB = collisionData.getPointOnB();
      FramePoint3D pointOnARootFrame = new FramePoint3D();
      FramePoint3D pointOnBRootFrame = new FramePoint3D();
      pointOnARootFrame.setIncludingFrame(pointOnA);
      pointOnBRootFrame.setIncludingFrame(pointOnB);
      pointOnARootFrame.changeFrame(worldFrame);
      pointOnBRootFrame.changeFrame(worldFrame);

      FrameVector3D collisionAxis = new FrameVector3D();

      FrameVector3D normalOnA = collisionData.getNormalOnA();
      FrameVector3D normalOnB = collisionData.getNormalOnB();

      if (!normalOnA.containsNaN())
      {
         collisionAxis.setIncludingFrame(normalOnA);
         collisionAxis.negate();
      }
      else if (!normalOnB.containsNaN())
      {
         collisionAxis.setIncludingFrame(normalOnB);
      }
      else
      {
         collisionAxis.setReferenceFrame(worldFrame);
         collisionAxis.sub(pointOnBRootFrame, pointOnARootFrame);
      }

      System.out.println(collisionData);

   }

}
