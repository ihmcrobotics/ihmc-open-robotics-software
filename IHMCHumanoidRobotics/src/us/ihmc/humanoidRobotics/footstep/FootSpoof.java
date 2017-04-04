package us.ihmc.humanoidRobotics.footstep;

import java.util.ArrayList;
import java.util.List;

import sun.reflect.generics.reflectiveObjects.NotImplementedException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
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
   private final List<FramePoint> contactPoints = new ArrayList<FramePoint>();
   private final List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>();
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
      soleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(name + "soleFrame", ankle.getFrameAfterJoint(), transformToAnkle);

      for (Point2D contactPointInSoleFrame : contactPoints2dInSoleFrame)
      {
         FramePoint point = new FramePoint(soleFrame, contactPointInSoleFrame.getX(), contactPointInSoleFrame.getY(), 0.0);
         contactPoints.add(point);
         contactPoints2d.add(point.toFramePoint2d());
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
      soleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(name + "soleFrame", ankle.getFrameAfterJoint(), transformToAnkle);
      FramePoint point1 = new FramePoint(soleFrame, new Point3D(footForward, footHalfWidth, 0.0));
      FramePoint point2 = new FramePoint(soleFrame, new Point3D(footForward, -footHalfWidth, 0.0));
      FramePoint point3 = new FramePoint(soleFrame, new Point3D(-footBack, -footHalfWidth, 0.0));
      FramePoint point4 = new FramePoint(soleFrame, new Point3D(-footBack, footHalfWidth, 0.0));
      contactPoints.add(point1);
      contactPoints.add(point2);
      contactPoints.add(point3);
      contactPoints.add(point4);
      contactPoints2d.add(point1.toFramePoint2d());
      contactPoints2d.add(point2.toFramePoint2d());
      contactPoints2d.add(point3.toFramePoint2d());
      contactPoints2d.add(point4.toFramePoint2d());

      totalNumberOfContactPoints = contactPoints.size();

      this.coefficientOfFriction = coefficientOfFriction;
   }

   public void setPose(FramePoint position, FrameOrientation orientation)
   {
      shinFrame.setPoseAndUpdate(position, orientation);
   }

   public void translate(double x, double y, double z)
   {
      shinFrame.translateAndUpdate(x, y, z);
   }

   public void setSoleFrame(FramePoint position, FrameOrientation orientation)
   {
      position.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      orientation.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setSoleFrame(new FramePose(ReferenceFrame.getWorldFrame(), position.getPoint(), orientation.getQuaternionCopy()));
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

   public List<FramePoint> getContactPointsCopy()
   {
      List<FramePoint> ret = new ArrayList<>();
      for (int i = 0; i < contactPoints.size(); i++)
      {
         ret.add(new FramePoint(contactPoints.get(i)));
      }
      return ret;
   }

   public ReferenceFrame getFrameAfterParentJoint()
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

   public List<FramePoint2d> getContactPoints2d()
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
