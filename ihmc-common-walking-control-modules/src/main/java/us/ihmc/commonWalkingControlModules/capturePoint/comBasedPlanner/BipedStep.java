package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public class BipedStep
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private RobotSide robotSide = RobotSide.LEFT;
   private FixedFramePose3DBasics goalPose = new FramePose3D(worldFrame);
   private final RecyclingArrayList<Point2D> predictedContactPoints = new RecyclingArrayList<>(6, Point2D.class);
   private double swingHeight = 0.0;

   public BipedStep()
   {
      predictedContactPoints.clear();
   }

   public BipedStep(RobotSide robotSide, FramePose3D goalPose, double groundClearance)
   {
      this(robotSide, goalPose, groundClearance, null);
   }

   public BipedStep(RobotSide robotSide, FramePose3D goalPose, double groundClearance, List<Point2D> predictedContactPoints)
   {
      setRobotSide(robotSide);
      setGoalPose(goalPose);
      setSwingHeight(groundClearance);
      setPredictedContactPoints(predictedContactPoints);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public FramePose3DReadOnly getGoalPose()
   {
      return goalPose;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public List<Point2D> getPredictedContactPoints()
   {
      if (predictedContactPoints.isEmpty())
      {
         return null;
      }
      return predictedContactPoints;
   }

   public void set(BipedStep other)
   {
      setRobotSide(other.getRobotSide());
      setGoalPose(other.getGoalPose());
      setSwingHeight(other.getSwingHeight());
      setPredictedContactPoints(other.getPredictedContactPoints());
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      this.goalPose.setMatchingFrame(goalPose);
   }

   public void setGoalPose(FramePoint3DReadOnly goalPosition, FrameOrientation3DReadOnly goalOrientation)
   {
      this.goalPose.set(goalPosition, goalOrientation);
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setPredictedContactPoints(List<? extends Point2DReadOnly> contactPointList)
   {
      predictedContactPoints.clear();

      if (contactPointList == null)
      {
         return;
      }

      for (int i = 0; i < contactPointList.size(); i++)
      {
         Point2DReadOnly point = contactPointList.get(i);
         this.predictedContactPoints.add().set(point);
      }
   }

   @Override
   public String toString()
   {
      String string = super.toString();
      string += "\nrobotSide: " + getRobotSide();
      string += "\ngoalPose:" + getGoalPose();
      string += "\nswingHeight: " + getSwingHeight();
      List<Point2D> predictedContacts = getPredictedContactPoints();
      if (predictedContacts != null)
         string += "\npredictedContacts: " + predictedContacts;
      return string;
   }
}
