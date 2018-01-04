package us.ihmc.humanoidRobotics.footstep;

import java.util.ArrayList;
import java.util.List;

import sun.reflect.generics.reflectiveObjects.NotImplementedException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

/**
 * USE ONLY FOR TEST!
 *
 */
public class FootSpoof implements ContactablePlaneBody
{
   private final InverseDynamicsJoint ankle;
   private final RigidBody shin;
   private final RigidBody foot;
   private final PoseReferenceFrame shinFrame;
   private final ReferenceFrame soleFrame;
   private final List<FramePoint3D> contactPoints = new ArrayList<FramePoint3D>();
   private final List<FramePoint2D> contactPoints2d = new ArrayList<FramePoint2D>();
   private final double coefficientOfFriction;
   private final int totalNumberOfContactPoints;

   public FootSpoof(String name)
   {
      this(name, -0.15, 0.02, 0.21, 0.1, 0.05, 0.05, 0.0);
   }

   public FootSpoof(String name, double xToAnkle, double yToAnkle, double zToAnkle, List<Point2D> contactPoints2dInSoleFrame,
                    double coefficientOfFriction)
   {
      RigidBodyTransform transformToAnkle = new RigidBodyTransform();
      transformToAnkle.setTranslation(new Vector3D(-xToAnkle, -yToAnkle, -zToAnkle));

//    if(FootstepUtilsTest.DEBUG_TESTS)
//       System.out.println("FootSpoof: making transform from plane to ankle equal to "+transformToAnkle);

      shinFrame = new PoseReferenceFrame(name + "ShinFrame", ReferenceFrame.getWorldFrame());
      this.shin = new RigidBody(name, shinFrame);
      this.ankle = ScrewTools.addRevoluteJoint(name + "Ankle", shin, new RigidBodyTransform(), new Vector3D(0.0, 1.0, 0.0));
      this.foot = ScrewTools.addRigidBody(name, ankle, new Matrix3D(), 1.0, new RigidBodyTransform());
      soleFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(name + "soleFrame", ankle.getFrameAfterJoint(), transformToAnkle);

      for (Point2D contactPointInSoleFrame : contactPoints2dInSoleFrame)
      {
         FramePoint3D point = new FramePoint3D(soleFrame, contactPointInSoleFrame.getX(), contactPointInSoleFrame.getY(), 0.0);
         contactPoints.add(point);
         contactPoints2d.add(new FramePoint2D(point));
      }

      totalNumberOfContactPoints = contactPoints.size();

      this.coefficientOfFriction = coefficientOfFriction;
   }

   public FootSpoof(String name, double xToAnkle, double yToAnkle, double zToAnkle, double footForward, double footBack, double footHalfWidth,
                    double coefficientOfFriction)
   {
      RigidBodyTransform transformToAnkle = new RigidBodyTransform();
      transformToAnkle.setTranslation(new Vector3D(-xToAnkle, -yToAnkle, -zToAnkle));

//    if(FootstepUtilsTest.DEBUG_TESTS)
//       System.out.println("FootSpoof: making transform from plane to ankle equal to "+transformToAnkle);

      shinFrame = new PoseReferenceFrame(name + "ShinFrame", ReferenceFrame.getWorldFrame());
      this.shin = new RigidBody(name, shinFrame);
      this.ankle = ScrewTools.addRevoluteJoint(name + "Ankle", shin, new RigidBodyTransform(), new Vector3D(0.0, 1.0, 0.0));
      this.foot = ScrewTools.addRigidBody(name, ankle, new Matrix3D(), 1.0, new RigidBodyTransform());
      soleFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(name + "soleFrame", ankle.getFrameAfterJoint(), transformToAnkle);
      FramePoint3D point1 = new FramePoint3D(soleFrame, new Point3D(footForward, footHalfWidth, 0.0));
      FramePoint3D point2 = new FramePoint3D(soleFrame, new Point3D(footForward, -footHalfWidth, 0.0));
      FramePoint3D point3 = new FramePoint3D(soleFrame, new Point3D(-footBack, -footHalfWidth, 0.0));
      FramePoint3D point4 = new FramePoint3D(soleFrame, new Point3D(-footBack, footHalfWidth, 0.0));
      contactPoints.add(point1);
      contactPoints.add(point2);
      contactPoints.add(point3);
      contactPoints.add(point4);
      contactPoints2d.add(new FramePoint2D(point1));
      contactPoints2d.add(new FramePoint2D(point2));
      contactPoints2d.add(new FramePoint2D(point3));
      contactPoints2d.add(new FramePoint2D(point4));

      totalNumberOfContactPoints = contactPoints.size();

      this.coefficientOfFriction = coefficientOfFriction;
   }

   public void setPose(FramePoint3D position, FrameQuaternion orientation)
   {
      shinFrame.setPoseAndUpdate(position, orientation);
   }

   public void translate(double x, double y, double z)
   {
      shinFrame.translateAndUpdate(x, y, z);
   }

   public void setSoleFrame(FramePoint3D position, FrameQuaternion orientation)
   {
      position.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      orientation.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setSoleFrame(new FramePose(position, orientation));
   }

   public void setSoleFrame(FramePose newSolePoseInWorldFrame)
   {
      newSolePoseInWorldFrame.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      RigidBodyTransform transformFromSoleToWorld = new RigidBodyTransform();
      newSolePoseInWorldFrame.getPose(transformFromSoleToWorld);

      RigidBodyTransform transformFromShinToWorld = new RigidBodyTransform();
      transformFromShinToWorld.set(transformFromSoleToWorld);
      transformFromShinToWorld.multiply(shinFrame.getTransformToDesiredFrame(soleFrame));

      shinFrame.setPoseAndUpdate(transformFromShinToWorld);
   }

   public String getName()
   {
      return foot.getName();
   }

   public RigidBody getRigidBody()
   {
      return foot;
   }

   public List<FramePoint3D> getContactPointsCopy()
   {
      List<FramePoint3D> ret = new ArrayList<>();
      for (int i = 0; i < contactPoints.size(); i++)
      {
         ret.add(new FramePoint3D(contactPoints.get(i)));
      }
      return ret;
   }

   public MovingReferenceFrame getFrameAfterParentJoint()
   {
      return ankle.getFrameAfterJoint();
   }

   public boolean inContact()
   {
      return false;
   }

   public ReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }

   public List<FramePoint2D> getContactPoints2d()
   {
      return contactPoints2d;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public int getTotalNumberOfContactPoints()
   {
      return totalNumberOfContactPoints;
   }

   @Override
   public void setSoleFrameTransformFromParentJoint(RigidBodyTransform transform)
   {
      throw new NotImplementedException();
   }
}
