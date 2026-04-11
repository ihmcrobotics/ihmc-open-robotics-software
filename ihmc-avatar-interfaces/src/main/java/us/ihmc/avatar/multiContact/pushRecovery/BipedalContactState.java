package us.ihmc.avatar.multiContact.pushRecovery;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.List;

public class BipedalContactState
{
   private final SideDependentList<FramePose3D> footZUpPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<PoseReferenceFrame> footZUpFrames = new SideDependentList<>();

   private final FramePose3D midFeetZUpPose = new FramePose3D();
   private final PoseReferenceFrame midFeetZUpFrame = new PoseReferenceFrame("midFeetZUpFrame", ReferenceFrame.getWorldFrame());

   private final RobotContactPointParameters<RobotSide> contactPointParameters;
   private final FrameConvexPolygon2D nominalCoPPolygon = new FrameConvexPolygon2D();

   private final SideDependentList<FramePoint3D> footPoints = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());

   private double stanceWidth;
   private double stanceLength;
   private double stanceYaw;

   public BipedalContactState(RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      this.contactPointParameters = contactPointParameters;

      for (RobotSide robotSide : RobotSide.values)
      {
         footZUpFrames.put(robotSide,
                           new PoseReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + "SoleZUpFrame", ReferenceFrame.getWorldFrame()));
      }
   }

   public void setToSingleSupport(RobotSide robotSide, FramePose3DReadOnly footZUpPose)
   {
      footZUpPoses.get(robotSide).setIncludingFrame(footZUpPose);
      footZUpPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());
      footZUpFrames.get(robotSide).setPoseAndUpdate(footZUpPoses.get(robotSide));

      midFeetZUpPose.set(footZUpPoses.get(robotSide));
      midFeetZUpFrame.setPoseAndUpdate(midFeetZUpPose);

      footZUpPoses.get(robotSide.getOppositeSide()).setToNaN();

      nominalCoPPolygon.clear();
      addFootContactPoints(robotSide);
      nominalCoPPolygon.update();
   }

   public void setToDoubleSupport(FramePose3DReadOnly leftFootZUpPose, FramePose3DReadOnly rightFootZUpPose)
   {
      footZUpPoses.get(RobotSide.LEFT).setIncludingFrame(leftFootZUpPose);
      footZUpPoses.get(RobotSide.RIGHT).setIncludingFrame(rightFootZUpPose);

      nominalCoPPolygon.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         footZUpPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());
         footZUpFrames.get(robotSide).setPoseAndUpdate(footZUpPoses.get(robotSide));
         addFootContactPoints(robotSide);
      }
      nominalCoPPolygon.update();

      midFeetZUpPose.interpolate(footZUpPoses.get(RobotSide.LEFT), footZUpPoses.get(RobotSide.RIGHT), 0.5);
      midFeetZUpFrame.setPoseAndUpdate(midFeetZUpPose);

      stanceYaw = EuclidCoreTools.angleDifferenceMinusPiToPi(leftFootZUpPose.getYaw(), rightFootZUpPose.getYaw());

      for (RobotSide robotSide : RobotSide.values)
      {
         footPoints.get(robotSide).setToZero(footZUpFrames.get(robotSide));
         footPoints.get(robotSide).changeFrame(midFeetZUpFrame);
      }

      stanceLength = footPoints.get(RobotSide.LEFT).getX() - footPoints.get(RobotSide.RIGHT).getX();
      stanceWidth = footPoints.get(RobotSide.LEFT).getY() - footPoints.get(RobotSide.RIGHT).getY();
   }

   public boolean isFootInContact(RobotSide robotSide)
   {
      return !footZUpPoses.get(robotSide).containsNaN();
   }

   public boolean isDoubleSupport()
   {
      return isFootInContact(RobotSide.LEFT) && isFootInContact(RobotSide.RIGHT);
   }

   public PoseReferenceFrame getMidFeetZUpFrame()
   {
      return midFeetZUpFrame;
   }

   public FrameConvexPolygon2DReadOnly getNominalCoPPolygon()
   {
      return nominalCoPPolygon;
   }

   public double getStanceWidth()
   {
      return stanceWidth;
   }

   public double getStanceLength()
   {
      return stanceLength;
   }

   public double getStanceYaw()
   {
      return stanceYaw;
   }

   private void addFootContactPoints(RobotSide robotSide)
   {
      List<Point2D> contactPoints = contactPointParameters.getFootContactPoints().get(robotSide);

      for (int i = 0; i < contactPoints.size(); i++)
      {
         nominalCoPPolygon.addVertexMatchingFrame(footZUpFrames.get(robotSide), contactPoints.get(i), false);
      }
   }
}
