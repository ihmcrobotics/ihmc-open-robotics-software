package us.ihmc.humanoidRobotics.footstep;

import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FramePose;
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

      FramePose footFramePose = new FramePose(endEffector.getSoleFrame());
      footFramePose.changeFrame(worldFrame);
      boolean trustHeight = false;

      Footstep footstep = new Footstep(side, footFramePose, trustHeight);

      return footstep;
   }

   public static Footstep generateStandingFootstep(RobotSide side, RigidBody foot, ReferenceFrame soleFrame)
   {
      FramePose footFramePose = new FramePose(soleFrame);
      footFramePose.changeFrame(worldFrame);
      boolean trustHeight = false;

      Footstep footstep = new Footstep(side, footFramePose, trustHeight);

      return footstep;
   }

   //used once
   public static FramePoint3D getCenterOfFootstep(Footstep footstep)
   {
      return getCenterOfFootstep(footstep, RobotSide.LEFT, 0.0, 0.0);
   }

   public static FramePoint3D getCenterOfFootstep(Footstep stance, RobotSide side, double centimetersForwardFromCenter, double centimetersInFromCenter)
   {
      FramePoint3D stanceCenter = new FramePoint3D(stance.getSoleReferenceFrame());
      stanceCenter.getReferenceFrame().checkReferenceFrameMatch(stance.getSoleReferenceFrame());
      stanceCenter.setX(stanceCenter.getX() + centimetersForwardFromCenter);
      stanceCenter.setY(stanceCenter.getY() + side.negateIfLeftSide(centimetersInFromCenter));
      stanceCenter.changeFrame(worldFrame);

      return stanceCenter;
   }

   public static FramePoint3D getCenterOfPredictedContactPointsInFootstep(Footstep footstep, RobotSide side, double centimetersForwardFromCenter, double centimetersInFromCenter)
   {
      Point2D footstepCenter;

      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
      if (predictedContactPoints != null)
      {
         ConvexPolygon2D contactPolygon = new ConvexPolygon2D(predictedContactPoints);
         footstepCenter = new Point2D(contactPolygon.getCentroid());
      }
      else
      {
         footstepCenter = new Point2D();
      }

      footstepCenter.setX(footstepCenter.getX() + centimetersForwardFromCenter);
      footstepCenter.setY(footstepCenter.getY() + side.negateIfLeftSide(centimetersInFromCenter));

      FramePoint3D footstepCenter3d = new FramePoint3D(footstep.getSoleReferenceFrame(), footstepCenter.getX(), footstepCenter.getY(), 0.0);
      footstepCenter3d.changeFrame(worldFrame);

      return footstepCenter3d;
   }

   //used in 1 other class
   public static RobotSide getFrontFootRobotSideBasedOnSoleCenters(SideDependentList<? extends ContactablePlaneBody> bipedFeet, FramePoint3D destination)
   {
      FramePoint3D destinationInWorld = new FramePoint3D(destination);
      destinationInWorld.changeFrame(worldFrame);
      SideDependentList<Double> distances = new SideDependentList<Double>();
      FramePoint3D footCenter = new FramePoint3D();
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
   public static RobotSide getFrontFootRobotSideFromFootsteps(SideDependentList<Footstep> footSteps, FramePoint3D destination)
   {
      FramePoint3D destinationInWorld = new FramePoint3D(destination);
      destinationInWorld.changeFrame(worldFrame);
      SideDependentList<Double> distances = new SideDependentList<Double>();
      for (RobotSide side : RobotSide.values)
      {
         FramePoint3D footstepPosition = new FramePoint3D();
         footSteps.get(side).getPosition(footstepPosition);
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
}
