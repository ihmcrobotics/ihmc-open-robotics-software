package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class MultiContactSupportRegionSolverInputExamples
{
   public static MultiContactFrictionBasedSupportRegionSolverInput createTriangleFlatGround()
   {
      return createTriangleInput(0.0, 0.0, 0.0);
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createTriangleTiltedOutSlightly()
   {
      double angle = Math.toRadians(30.0);
      return createTriangleInput(angle, angle, angle);
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createTriangleTiltedOutALot()
   {
      double angle = Math.toRadians(50.0);
      return createTriangleInput(angle, angle, angle);
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createTriangleOneTiltedFullyOut()
   {
      double theta0 = Math.toRadians(90.0);
      double theta2 = Math.toRadians(0.0);
      double theta1 = Math.toRadians(0.0);
      return createTriangleInput(theta0, theta1, theta2);
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createTriangleOneTiltedFullyIn()
   {
      double theta0 = Math.toRadians(-90.0);
      double theta2 = Math.toRadians(0.0);
      double theta1 = Math.toRadians(0.0);
      return createTriangleInput(theta0, theta1, theta2);
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createFlatSquare()
   {
      MultiContactFrictionBasedSupportRegionSolverInput input = new MultiContactFrictionBasedSupportRegionSolverInput();
      input.addContactPoint(new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.5, -0.5, 0.0), new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis3D.Z));
      input.addContactPoint(new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.5, 0.5, 0.0), new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis3D.Z));
      input.addContactPoint(new FramePoint3D(ReferenceFrame.getWorldFrame(),  0.5, -0.5, 0.0), new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis3D.Z));
      input.addContactPoint(new FramePoint3D(ReferenceFrame.getWorldFrame(),  0.5, 0.5, 0.0), new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis3D.Z));
      return input;
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createSingleFlatContactPoint(double x, double y)
   {
      MultiContactFrictionBasedSupportRegionSolverInput input = new MultiContactFrictionBasedSupportRegionSolverInput();
      input.addContactPoint(new FramePoint3D(ReferenceFrame.getWorldFrame(), x, y, 0.0), new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis3D.Z));
      return input;
   }

   private static MultiContactFrictionBasedSupportRegionSolverInput createTriangleInput(double... angles)
   {
      MultiContactFrictionBasedSupportRegionSolverInput input = new MultiContactFrictionBasedSupportRegionSolverInput();

      double distance = 1.0;
      for (int i = 0; i < 3; i++)
      {
         double theta = i * 2.0 * Math.PI / 3.0;
         FramePoint3D contactPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), distance * Math.cos(theta), distance * Math.sin(theta), 0.0);

         Vector3D axis = new Vector3D(contactPoint);
         axis.normalize();
         axis.set(-axis.getY(), axis.getX(), 0.0);

         AxisAngle axisAngle = new AxisAngle(axis, angles[i]);
         FrameVector3D normal = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
         axisAngle.transform(normal);

         input.addContactPoint(contactPoint, normal);
      }

      return input;
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createBipedFeet()
   {
      return createBipedFeet(0.0, 0.0, 0.0);
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createBipedFeet(double dx, double dy, double dz)
   {
      MultiContactFrictionBasedSupportRegionSolverInput input = new MultiContactFrictionBasedSupportRegionSolverInput();

      double footWidth = 0.1;
      double footLength = 0.2;
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(0.5 * footLength, 0.5 * footWidth);
      footPolygon.addVertex(0.5 * footLength, -0.5 * footWidth);
      footPolygon.addVertex(-0.5 * footLength, 0.5 * footWidth);
      footPolygon.addVertex(-0.5 * footLength, -0.5 * footWidth);
      footPolygon.update();

      FramePose3D leftFootPose = new FramePose3D();
      FramePose3D rightFootPose = new FramePose3D();

      leftFootPose.getPosition().set(dx, dy + 0.25, dz);
      leftFootPose.getOrientation().setToRollOrientation(Math.toRadians(30.0));

      rightFootPose.getPosition().set(dx, dy + -0.25, dz - 0.1);
      rightFootPose.getOrientation().setYawPitchRoll(Math.toRadians(-20.0), Math.toRadians(20.0), 0.0);

      SideDependentList<FramePose3D> footPoses = new SideDependentList<>(leftFootPose, rightFootPose);
      PoseReferenceFrame contactPointFrame = new PoseReferenceFrame("contactPointFrame", ReferenceFrame.getWorldFrame());

      for (RobotSide robotSide : RobotSide.values())
      {
         for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
         {
            addContactPoint(contactPointFrame, footPoses.get(robotSide), input, new Point3D(footPolygon.getVertex(i)));
         }
      }

      return input;
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createBipedFeetWithSingleHandhold()
   {
      MultiContactFrictionBasedSupportRegionSolverInput input = new MultiContactFrictionBasedSupportRegionSolverInput();

      double footWidth = 0.1;
      double footLength = 0.2;
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(0.5 * footLength, 0.5 * footWidth);
      footPolygon.addVertex(0.5 * footLength, -0.5 * footWidth);
      footPolygon.addVertex(-0.5 * footLength, 0.5 * footWidth);
      footPolygon.addVertex(-0.5 * footLength, -0.5 * footWidth);
      footPolygon.update();

      FramePose3D leftFootPose = new FramePose3D();
      FramePose3D rightFootPose = new FramePose3D();

      leftFootPose.getPosition().set(0.0, 0.25, 0.0);
      leftFootPose.getOrientation().setToRollOrientation(Math.toRadians(30.0));

      rightFootPose.getPosition().set(0.0, -0.25, -0.1);
      rightFootPose.getOrientation().setYawPitchRoll(Math.toRadians(-20.0), Math.toRadians(20.0), 0.0);

      SideDependentList<FramePose3D> footPoses = new SideDependentList<>(leftFootPose, rightFootPose);
      PoseReferenceFrame contactPointFrame = new PoseReferenceFrame("contactPointFrame", ReferenceFrame.getWorldFrame());

      for (RobotSide robotSide : RobotSide.values())
      {
         for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
         {
            addContactPoint(contactPointFrame, footPoses.get(robotSide), input, new Point3D(footPolygon.getVertex(i)));
         }
      }

      FramePose3D handPose = new FramePose3D();
      handPose.getPosition().set(0.8, 0.0, 0.85);
      handPose.getOrientation().setToPitchOrientation(Math.toRadians(-90.0));
      addContactPoint(contactPointFrame, handPose, input, null);

      return input;
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput createBipedFeetWithTwoHandholds()
   {
      MultiContactFrictionBasedSupportRegionSolverInput input = new MultiContactFrictionBasedSupportRegionSolverInput();

      double footWidth = 0.1;
      double footLength = 0.2;
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(0.5 * footLength, 0.5 * footWidth);
      footPolygon.addVertex(0.5 * footLength, -0.5 * footWidth);
      footPolygon.addVertex(-0.5 * footLength, 0.5 * footWidth);
      footPolygon.addVertex(-0.5 * footLength, -0.5 * footWidth);
      footPolygon.update();

      FramePose3D leftFootPose = new FramePose3D();
      FramePose3D rightFootPose = new FramePose3D();

      leftFootPose.getPosition().set(0.0, 0.25, 0.0);
      leftFootPose.getOrientation().setToRollOrientation(Math.toRadians(30.0));

      rightFootPose.getPosition().set(0.0, -0.25, -0.1);
      rightFootPose.getOrientation().setYawPitchRoll(Math.toRadians(-20.0), Math.toRadians(20.0), 0.0);

      SideDependentList<FramePose3D> footPoses = new SideDependentList<>(leftFootPose, rightFootPose);
      PoseReferenceFrame contactPointFrame = new PoseReferenceFrame("contactPointFrame", ReferenceFrame.getWorldFrame());

      for (RobotSide robotSide : RobotSide.values())
      {
         for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
         {
            addContactPoint(contactPointFrame, footPoses.get(robotSide), input, new Point3D(footPolygon.getVertex(i)));
         }
      }

      FramePose3D leftHandPose = new FramePose3D();
      leftHandPose.getPosition().set(0.8, -0.1, 0.9);
      leftHandPose.getOrientation().setToPitchOrientation(Math.toRadians(-90.0));
      addContactPoint(contactPointFrame, leftHandPose, input, null);

      FramePose3D rightHandPose = new FramePose3D();
      rightHandPose.getPosition().set(0.0, 0.5, 1.0);
      rightHandPose.getOrientation().setToRollOrientation(Math.toRadians(20.0));
      rightHandPose.getOrientation().appendPitchRotation(Math.toRadians(5.0));
      addContactPoint(contactPointFrame, rightHandPose, input, null);

      return input;
   }

   private static void addContactPoint(PoseReferenceFrame contactFrame, FramePose3D contactPose, MultiContactFrictionBasedSupportRegionSolverInput input, Point3D contactPointInLocal)
   {
      contactFrame.setPoseAndUpdate(contactPose);

      FramePoint3D contactPoint = new FramePoint3D(contactFrame);
      if (contactPointInLocal != null)
         contactPoint.set(contactPointInLocal);

      FrameVector3D normal = new FrameVector3D(contactFrame, Axis3D.Z);

      contactPoint.changeFrame(ReferenceFrame.getWorldFrame());
      normal.changeFrame(ReferenceFrame.getWorldFrame());

      input.addContactPoint(contactPoint, normal);
   }
}
