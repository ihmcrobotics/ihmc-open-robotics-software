package us.ihmc.humanoidRobotics.footstep;

import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * @author Sylvain
 */
@Deprecated
public class FootstepUtils
{
   private final static boolean DEBUG = false;

   static
   {
      if (DEBUG)
         System.out.println("FootstepUtils: DEBUG flag is true.");
   }

   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static Footstep generateStandingFootstep(RobotSide side, SideDependentList<? extends ContactablePlaneBody> bipedFeet)
   {
      ContactablePlaneBody endEffector = bipedFeet.get(side);

      FramePose footFramePose = new FramePose(endEffector.getFrameAfterParentJoint());
      footFramePose.changeFrame(worldFrame);
      boolean trustHeight = false;

      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footFramePose);
      Footstep footstep = new Footstep(endEffector.getRigidBody(), side, endEffector.getSoleFrame(), footstepPoseFrame, trustHeight);

      return footstep;
   }

   public static Footstep generateStandingFootstep(RobotSide side, RigidBody foot, ReferenceFrame soleFrame)
   {
      FramePose footFramePose = new FramePose(foot.getParentJoint().getFrameAfterJoint());
      footFramePose.changeFrame(worldFrame);
      boolean trustHeight = false;

      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footFramePose);
      Footstep footstep = new Footstep(foot, side, soleFrame, footstepPoseFrame, trustHeight);

      return footstep;
   }

   //used once
   public static FramePoint getCenterOfFootstep(Footstep footstep)
   {
      return getCenterOfFootstep(footstep, RobotSide.LEFT, 0.0, 0.0);
   }

   public static FramePoint getCenterOfFootstep(Footstep stance, RobotSide side, double centimetersForwardFromCenter, double centimetersInFromCenter)
   {
      FramePoint stanceCenter = new FramePoint(stance.getSoleReferenceFrame());
      stanceCenter.getReferenceFrame().checkReferenceFrameMatch(stance.getSoleReferenceFrame());
      stanceCenter.setX(stanceCenter.getX() + centimetersForwardFromCenter);
      stanceCenter.setY(stanceCenter.getY() + side.negateIfLeftSide(centimetersInFromCenter));
      stanceCenter.changeFrame(worldFrame);

      return stanceCenter;
   }
   
   public static FramePoint getCenterOfPredictedContactPointsInFootstep(Footstep footstep, RobotSide side, double centimetersForwardFromCenter, double centimetersInFromCenter)
   {
      Point2D footstepCenter;
      
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
      if (predictedContactPoints != null)
      {
         ConvexPolygon2d contactPolygon = new ConvexPolygon2d(predictedContactPoints);
         footstepCenter = new Point2D(contactPolygon.getCentroid());
      }
      else
      {
         footstepCenter = new Point2D();
      }
      
      footstepCenter.setX(footstepCenter.getX() + centimetersForwardFromCenter);
      footstepCenter.setY(footstepCenter.getY() + side.negateIfLeftSide(centimetersInFromCenter));
      
      FramePoint footstepCenter3d = new FramePoint(footstep.getSoleReferenceFrame(), footstepCenter.getX(), footstepCenter.getY(), 0.0);
      footstepCenter3d.changeFrame(worldFrame);

      return footstepCenter3d;
   }

   //used in 1 other class
   public static RobotSide getFrontFootRobotSideBasedOnSoleCenters(SideDependentList<? extends ContactablePlaneBody> bipedFeet, FramePoint destination)
   {
      FramePoint destinationInWorld = new FramePoint(destination);
      destinationInWorld.changeFrame(worldFrame);
      SideDependentList<Double> distances = new SideDependentList<Double>();
      FramePoint footCenter = new FramePoint();
      for (RobotSide side : RobotSide.values)
      {
         footCenter.setToZero(bipedFeet.get(side).getSoleFrame());
         footCenter.changeFrame(worldFrame);
         distances.set(side, footCenter.distanceSquared(destinationInWorld));
      }

      RobotSide frontSide = (distances.get(RobotSide.LEFT) <= distances.get(RobotSide.RIGHT)) ? RobotSide.LEFT : RobotSide.RIGHT;

      return frontSide;
   }

   //used once
   public static RobotSide getFrontFootRobotSideFromFootsteps(SideDependentList<Footstep> footSteps, FramePoint destination)
   {
      FramePoint destinationInWorld = new FramePoint(destination);
      destinationInWorld.changeFrame(worldFrame);
      SideDependentList<Double> distances = new SideDependentList<Double>();
      for (RobotSide side : RobotSide.values)
      {
         FramePoint footstepPosition = new FramePoint();
         footSteps.get(side).getPositionIncludingFrame(footstepPosition);
         footstepPosition.changeFrame(worldFrame);
         distances.set(side, new Double(footstepPosition.distanceSquared(destinationInWorld)));
      }

      RobotSide frontSide = (distances.get(RobotSide.LEFT) <= distances.get(RobotSide.RIGHT)) ? RobotSide.LEFT : RobotSide.RIGHT;

      return frontSide;
   }

   //used once
   public static double getHeading(ReferenceFrame ankleFrame)
   {
      RigidBodyTransform transform = ankleFrame.getTransformToDesiredFrame(worldFrame);
      Vector3D xVector = new Vector3D(1.0, 0.0, 0.0);
      transform.transform(xVector);
      xVector.sub(new Vector3D(0.0, 0.0, -xVector.getZ()));
      xVector.normalize();

      if (Double.isNaN(xVector.getX()))
      {
         throw new RuntimeException("FootstepUtils: Footsteps cannot face directly upwards");
      }

      double cosTheta = xVector.dot(new Vector3D(1.0, 0.0, 0.0));
      double sinTheta = xVector.dot(new Vector3D(0.0, 1.0, 0.0));
      double theta = Math.atan2(sinTheta, cosTheta);

      return theta;
   }

   public static List<FramePoint> calculateExpectedContactPoints(Footstep atFootstep, ContactablePlaneBody foot)
   {
      if (!atFootstep.getBody().getName().equals(foot.getRigidBody().getName()))
         throw new RuntimeException("The RigidBodies are not the same.");
      
      List<FramePoint> ret = foot.getContactPointsCopy();
      
      ReferenceFrame soleFrame = atFootstep.getSoleReferenceFrame();
      
      for (int i = 0; i < ret.size(); i++)
      {
         FramePoint contactPoint = ret.get(i);
         contactPoint.checkReferenceFrameMatch(foot.getSoleFrame());
         contactPoint.setIncludingFrame(soleFrame, contactPoint.getPoint());
      }

      return ret;
   }
}
