package us.ihmc.darpaRoboticsChallenge.footstepGenerator;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTools;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTrajectoryHeightCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepValidityMetric;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnapper;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;

/**
 * Created by agrabertilton on 2/19/15.
 */
public class PathToFootstepGenerator
{
   private static final Vector3d verticalUnitVector = new Vector3d(0.0, 0.0, 1.0);
   private FootstepSnapper snapper;
   private SwingTrajectoryHeightCalculator heightCalculator;
   FootstepValidityMetric validityMetric;
   private HeightMapWithPoints heightMap;

   // TODO @Agraber make non-atlas specific.
   private double stepLength = 0.6;
   private double stepDecrement = 0.025;
   private double verticalEquivalenceScale = 1.0;    // 1cm vertical distance is equivalent to 1cm horizontal distance;
   private double maxYaw = Math.PI / 4;
   protected double horizontalDistance = 0.25 / 2.0;
   private double downwardsDistanceGain = 2.0;
   private double maxZHeightChange = 0.30;

   private boolean initialized = false;
   private FootstepOverheadPath overheadPath;
   private SideDependentList<FootstepData> originalFeet = new SideDependentList<>();

   public PathToFootstepGenerator(SwingTrajectoryHeightCalculator heightCalculator, FootstepSnapper footstepSnapper, FootstepValidityMetric validityMetric,
                                  HeightMapWithPoints heightMap)
   {
      snapper = footstepSnapper;
      this.heightCalculator = heightCalculator;
      this.heightMap = heightMap;
      this.validityMetric = validityMetric;
   }

   public void setHeightMap(HeightMapWithPoints heightMap)
   {
      this.heightMap = heightMap;
   }

   public boolean hasHeightMap()
   {
      if (heightMap == null)
      {
         return false;
      }

      return true;
   }

   /**
    *
    * @param originalFeet the original sole placement of the feet, (not the ankle position)
    * @param overheadPath the overhead path for the robot to place footsteps along
    */
   public void initialize(SideDependentList<FootstepData> originalFeet, FootstepOverheadPath overheadPath)
   {
      overheadPath.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      this.originalFeet = originalFeet;
      this.overheadPath = overheadPath;

      initialized = true;
   }

   public ArrayList<Footstep> generateFoootStepsAlongPath(RobotSide firstStepSide, SideDependentList<ContactablePlaneBody> contactableFeet)
   {
      List<FootstepData> generatedFootsteps = getStepsAlongPath(firstStepSide);

      // convert from footstepData to footsteps
      ArrayList<Footstep> footsteps = new ArrayList<>();
      for (FootstepData footstepData : generatedFootsteps)
      {
         Footstep footstep = FootstepTools.generateFootstepFromFootstepDataSole(footstepData, contactableFeet.get(footstepData.getRobotSide()));
         footsteps.add(footstep);
      }

      return footsteps;
   }

   public ArrayList<FootstepData> getStepsAlongPath(RobotSide firstStepSide)
   {
      if (!initialized)
      {
         throw new RuntimeException(this.getClass().getSimpleName() + ": Not initialized");
      }

//      if (!hasHeightMap())
//      {
//         throw new RuntimeException(this.getClass().getSimpleName() + ": No HeightMap");
//      }

      ArrayList<FootstepData> footstepList = new ArrayList<>();
      double totalDistance = overheadPath.getTotalDistance();

      FootstepData stanceFoot;
      FootstepData initialSwingFoot;


      SideDependentList<FootstepData> lastFeet = new SideDependentList<>();
      lastFeet.put(RobotSide.LEFT, originalFeet.get(RobotSide.LEFT));
      lastFeet.put(RobotSide.RIGHT, originalFeet.get(RobotSide.RIGHT));

      RobotSide currentSide = firstStepSide;
      double lastDistance = 0;
      double currentDistance = 0;
      double nextFootDistance;

      // preinitialization
      nextFootDistance = Math.min(totalDistance, currentDistance + stepLength);
      stanceFoot = lastFeet.get(currentSide.getOppositeSide());
      initialSwingFoot = lastFeet.get(currentSide);

      FramePose2d currentPose = overheadPath.getPoseAtDistance(currentDistance);
      FramePose2d nextPose = overheadPath.getPoseAtDistance(nextFootDistance);
      FootstepData currentFootstep = new FootstepData(currentSide, new Point3d(), new Quat4d());

      FramePose2d goalPose = overheadPath.getPoseAtDistance(totalDistance);

      boolean fullPathFound = false;
      boolean stepValid = true;
      Footstep.FootstepType qualifierType = Footstep.FootstepType.FULL_FOOTSTEP;
      boolean zeroFootstepTried = false;
      while (!fullPathFound)
      {
         if (stepValid)
         {
            lastDistance = currentDistance;
            currentDistance = Math.min(totalDistance, lastDistance + stepLength);
            stanceFoot = lastFeet.get(currentSide.getOppositeSide());
            initialSwingFoot = lastFeet.get(currentSide);
            currentFootstep = new FootstepData(currentSide, new Point3d(), new Quat4d());
            qualifierType = Footstep.FootstepType.FULL_FOOTSTEP;
         }
         else
         {
            // recompute at lower distance traveled
            currentDistance -= stepDecrement;

            if (currentDistance - lastDistance <= 1e-3)
            {
               if (!zeroFootstepTried)
               {
                  //try step in place footstep
                  currentDistance = lastDistance;
                  zeroFootstepTried = true;
               }else
               {
                  zeroFootstepTried = false;
                  // no valid footsteps available, reduce qualifications
                  qualifierType = getLessRestrictiveType(qualifierType);
                  currentDistance = Math.min(totalDistance, lastDistance + stepLength);

                  if (qualifierType == Footstep.FootstepType.BAD_FOOTSTEP)
                  {
                     break;    // no non-bad footsteps found
                  }
               }
            }
         }

         nextFootDistance = Math.min(totalDistance, currentDistance + stepLength);

         currentPose = overheadPath.getPoseAtDistance(currentDistance);
         nextPose = overheadPath.getPoseAtDistance(nextFootDistance);

         double footYaw = getYawForFoot(stanceFoot, currentPose, nextPose, maxYaw);

         Point2d currentPosition = getOffsetPosition(currentPose, currentSide, horizontalDistance, footYaw);

         // set to the prospective position
         double defaultHeight = stanceFoot.getLocation().getZ();
         currentFootstep.getLocation().set(currentPosition.x, currentPosition.y, defaultHeight);
         RotationTools.getQuaternionFromYawAndZNormal(footYaw, verticalUnitVector, currentFootstep.getOrientation());

         if (heightMap != null)
         {
            // snap to the ground in the prospective position
            Footstep.FootstepType footstepType = snapper.snapFootstep(currentFootstep, heightMap);
            if (!betterThanType(footstepType, qualifierType))
            {    // footstep not good enough, keep searching
               stepValid = false;

               continue;
            }

            //set swing height for the footstep
            double swingHeight = heightCalculator.getSwingHeight(initialSwingFoot, stanceFoot, currentFootstep, heightMap);
            if (swingHeight > 0.2)
            {
               currentFootstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE);
            }
            currentFootstep.setSwingHeight(swingHeight);
         }

         stepValid = validityMetric.footstepValid(initialSwingFoot, stanceFoot, currentFootstep);

         if (stepValid)
         {
            // footstep fine, add footstep
            footstepList.add(currentFootstep);
            lastFeet.put(currentFootstep.getRobotSide(), currentFootstep);
            currentSide = currentSide.getOppositeSide();

            if (isGoalFootstep(currentFootstep, goalPose, horizontalDistance))
            {
               fullPathFound = true;
            }
         }
      }

      if (fullPathFound)
      {
         // TODO Add square-up footstep
         RobotSide squareUpSide = currentSide;
         stanceFoot = lastFeet.get(currentSide.getOppositeSide());
         initialSwingFoot = lastFeet.get(currentSide);
         currentFootstep = new FootstepData(currentSide, new Point3d(), new Quat4d());
         qualifierType = Footstep.FootstepType.FULL_FOOTSTEP;

         nextFootDistance = Math.min(totalDistance, currentDistance + stepLength);
         currentPose = overheadPath.getPoseAtDistance(currentDistance);
         nextPose = overheadPath.getPoseAtDistance(nextFootDistance);

         double footYaw = getYawForFoot(stanceFoot, currentPose, nextPose, maxYaw);
         Point2d currentPosition = getOffsetPosition(currentPose, currentSide, horizontalDistance, footYaw);

         // set to the prospective position
         double defaultHeight = stanceFoot.getLocation().getZ();
         currentFootstep.getLocation().set(currentPosition.x, currentPosition.y, defaultHeight);
         RotationTools.getQuaternionFromYawAndZNormal(footYaw, verticalUnitVector, currentFootstep.getOrientation());

         if (heightMap != null)
         {
            // snap to the ground in the prospective position
            Footstep.FootstepType footstepType = snapper.snapFootstep(currentFootstep, heightMap);
            double swingHeight = heightCalculator.getSwingHeight(initialSwingFoot, stanceFoot, currentFootstep, heightMap);
            if (swingHeight > 0.2)
            {
               currentFootstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE);
            }
            currentFootstep.setSwingHeight(swingHeight);
         }

         stepValid = validityMetric.footstepValid(initialSwingFoot, stanceFoot, currentFootstep);

         if (stepValid)
         {
            // footstep fine, add footstep
            footstepList.add(currentFootstep);
            lastFeet.put(currentFootstep.getRobotSide(), currentFootstep);
         }
      }

      return footstepList;
   }

   protected boolean isGoalFootstep(FootstepData footstep, FramePose2d goalPose, double horizontalDistance)
   {
      RobotSide currentSide = footstep.getRobotSide();
      double currentYaw = RotationTools.getYawFromQuaternion(footstep.getOrientation());
      double finalYaw = goalPose.getYaw();
      Point2d finalPosition = getOffsetPosition(goalPose, currentSide, horizontalDistance, finalYaw);
      if (Math.abs(finalYaw - currentYaw) > 1e-10)
      {
         return false;
      }

      Point3d currentLocation = footstep.getLocation();
      if (Math.abs(finalPosition.getX() - currentLocation.getX()) > 1e-10)
      {
         return false;
      }

      if (Math.abs(finalPosition.getY() - currentLocation.getY()) > 1e-10)
      {
         return false;
      }

      return true;
   }

   private Footstep.FootstepType getLessRestrictiveType(Footstep.FootstepType type)
   {
      switch (type)
      {
         case FULL_FOOTSTEP :
            return Footstep.FootstepType.PARTIAL_FOOTSTEP;

         case PARTIAL_FOOTSTEP :
         case BAD_FOOTSTEP :
         default :
            return Footstep.FootstepType.BAD_FOOTSTEP;
      }
   }

   private boolean betterThanType(Footstep.FootstepType type, Footstep.FootstepType typeToCompareAgainst)
   {
      return type.ordinal() <= typeToCompareAgainst.ordinal();
   }

   private Point2d getOffsetPosition(FramePose2d currentPose, RobotSide currentSide, double horizontalDistance, double footYaw)
   {
      Point2d position = new Point2d(currentPose.getX(), currentPose.getY());
      double yaw = footYaw;
      double sign = (currentSide == RobotSide.LEFT) ? 1 : -1;

      // sign * horizontalDistance
      position.x += -1.0 * Math.sin(yaw) * sign * horizontalDistance;
      position.y += 1.0 * Math.cos(yaw) * sign * horizontalDistance;

      return position;
   }

   private double getYawForFoot(FootstepData stanceFoot, FramePose2d currentPose, FramePose2d nextPose, double maxYaw)
   {
      double stanceYaw = RotationTools.getYawFromQuaternion(stanceFoot.getOrientation());
      double prospectiveYaw = currentPose.getYaw();
      double dYaw = AngleTools.trimAngleMinusPiToPi(prospectiveYaw - stanceYaw);
      double sign = stanceFoot.getRobotSide().negateIfLeftSide(1.0);

      if (sign * dYaw < 0.0)
      {
         prospectiveYaw = stanceYaw;
      }

      if (sign * dYaw > maxYaw)
      {
         prospectiveYaw = stanceYaw + sign * maxYaw;
      }

      return prospectiveYaw;
   }
}
