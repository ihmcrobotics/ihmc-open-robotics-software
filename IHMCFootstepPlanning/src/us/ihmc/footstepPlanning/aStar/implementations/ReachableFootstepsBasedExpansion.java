package us.ihmc.footstepPlanning.aStar.implementations;

import java.util.HashSet;

import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeExpansion;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
/**
 * This class expands the node based on a fixed number of defined configurations.
 * The list of neighbours generated is robot-specific.
 * @author Shlok Agarwal
 *
 */
public class ReachableFootstepsBasedExpansion implements FootstepNodeExpansion
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   ReferenceFrame stanceFrame;
   RobotSide stepSide;
   double yawStanceFoot;
   HashSet<FootstepNode> neighbors;

   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      neighbors = new HashSet<>();

      /** Get reference frame of stance leg */
      FramePose stanceFootPose = new FramePose(worldFrame);
      stanceFootPose.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      stanceFootPose.setX(node.getX());
      stanceFootPose.setY(node.getY());
      yawStanceFoot = node.getYaw();
      stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);

      /** Get swing leg side */
      stepSide = node.getRobotSide().getOppositeSide();

      //addNeighborsForValkyrie();
      addNeighborsForAtlas();

      return neighbors;
   }

   /**
    * Adds offsets to the current swing foot position to find potential neighbors
    * @param xOffset
    * @param yOffset
    * @param yawOffset
    */
   public void addOffset(double xOffset, double yOffset, double yawOffset)
   {
      /** Based on stance foot side, step width sign would change*/
      double ySign = stepSide.negateIfRightSide(1.0);

      FramePose step = new FramePose(stanceFrame);
      step.setX(xOffset);
      step.setY(ySign * yOffset);
      step.changeFrame(worldFrame);

      neighbors.add(new FootstepNode(step.getX(), step.getY(), yawStanceFoot + ySign * yawOffset, stepSide));
   }

   /**
    * Adds footstep offsets for Atlas robot
    */
   public void addNeighborsForAtlas()
   {

      /** Side Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/    
      addOffset(0.0     ,      0.25     ,      0.0);
      addOffset(0.0     ,      0.25     ,      0.174);
      addOffset(0.0     ,      0.25     ,      0.392);

      addOffset(0.0     ,      0.15     ,      0.0);
      addOffset(0.0     ,      0.20     ,      0.0);
      addOffset(0.0     ,      0.30     ,      0.0);

      /** Forward Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/ 

      addOffset(0.05     ,      0.25     ,      0.0);
      addOffset(0.05     ,      0.25     ,      0.174);
      addOffset(0.05     ,      0.25     ,      0.392);

      addOffset(0.1     ,      0.25     ,      0.0);
      addOffset(0.1     ,      0.25     ,      0.174);
      addOffset(0.1     ,      0.25     ,      0.392);

      addOffset(0.2     ,      0.25     ,      0.0);
      addOffset(0.2     ,      0.25     ,      0.174);
      addOffset(0.2     ,      0.25     ,      0.392);

      addOffset(0.3     ,      0.25     ,      0.0);
      addOffset(0.3     ,      0.25     ,      0.174);
      addOffset(0.3     ,      0.25     ,      0.392);

      addOffset(0.4     ,      0.25     ,      0.0);
      addOffset(0.4     ,      0.25     ,      0.174);
      addOffset(0.4     ,      0.25     ,      0.392);

      /** Backward Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/ 

      addOffset(-0.05     ,      0.25     ,      0.0);
      addOffset(-0.05     ,      0.25     ,      0.174);
      addOffset(-0.05     ,      0.25     ,      0.392);

      addOffset(-0.1     ,      0.25     ,      0.0);
      addOffset(-0.1     ,      0.25     ,      0.174);
      addOffset(-0.1     ,      0.25     ,      0.392);

      addOffset(-0.2     ,      0.25     ,      0.0);
      addOffset(-0.2     ,      0.25     ,      0.174);
      addOffset(-0.2     ,      0.25     ,      0.392);

      addOffset(-0.3     ,      0.25     ,      0.0);
      addOffset(-0.3     ,      0.25     ,      0.174);
      addOffset(-0.3     ,      0.25     ,      0.392);

      addOffset(-0.4     ,      0.25     ,      0.0);
      addOffset(-0.4     ,      0.25     ,      0.174);
      addOffset(-0.4     ,      0.25     ,      0.392);

      /** Turn Step*/
      addOffset(-0.047     ,      0.25     ,      0.392);

   }

   /**
    * Adds footstep offsets for Valkyrie robot
    * Note: These values have been rigourously tested in simulation but not on real robot.
    */
   public void addNeighborsForValkyrie()
   {
      /** Backward Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/    
      addOffset(-0.2     ,     0.3     ,     -0.1);
      addOffset(-0.2     ,     0.3     ,     -0.2);
      addOffset(-0.2     ,     0.3     ,     -0.4);
      addOffset(-0.2     ,     0.2     ,        0);
      addOffset(-0.2     ,     0.3     ,      0.1);
      addOffset(-0.2     ,     0.3     ,      0.2);
      addOffset(-0.2     ,     0.3     ,      0.4);
      addOffset(-0.1     ,     0.3     ,        0);
      addOffset(-0.1     ,     0.3     ,     -0.1);
      addOffset(-0.1     ,     0.3     ,     -0.2);
      addOffset(-0.1     ,     0.3     ,     -0.3);
      addOffset(-0.1     ,     0.35    ,     -0.4);
      addOffset(-0.1     ,     0.3     ,      0.1);
      addOffset(-0.1     ,     0.3     ,      0.2);
      addOffset(-0.1     ,     0.3     ,      0.3);
      addOffset(-0.1     ,     0.35    ,      0.4);

      /** Side Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/ 
      addOffset(0        ,     0.3     ,        0);
      addOffset(0        ,     0.35    ,     -0.2);
      addOffset(0        ,     0.35    ,     -0.4);
      addOffset(0        ,     0.35    ,     -0.6);
      addOffset(0        ,     0.35    ,     -0.7);
      addOffset(0        ,     0.35    ,      0.2);
      addOffset(0        ,     0.35    ,      0.4);
      addOffset(0        ,     0.35    ,      0.6);
      addOffset(0        ,     0.35    ,      0.7);
      addOffset(0        ,     0.3     ,        0);

      /** Forward Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/ 
      addOffset(0.1      ,     0.3     ,     -0.1);
      addOffset(0.1      ,     0.3     ,     -0.2);
      addOffset(0.1      ,     0.3     ,     -0.3);
      addOffset(0.1      ,     0.35    ,     -0.4);
      addOffset(0.1      ,     0.3     ,      0.1);
      addOffset(0.1      ,     0.3     ,      0.2);
      addOffset(0.1      ,     0.3     ,      0.3);
      addOffset(0.1      ,     0.3     ,      0.4);
      addOffset(0.2      ,     0.2     ,        0);
      addOffset(0.2      ,     0.3     ,     -0.1);
      addOffset(0.2      ,     0.3     ,     -0.2);
      addOffset(0.2      ,     0.3     ,     -0.4);
      addOffset(0.2      ,     0.3     ,      0.1);
      addOffset(0.2      ,     0.3     ,      0.2);
      addOffset(0.2      ,     0.3     ,      0.4);
      addOffset(0.3      ,     0.2     ,        0);
      addOffset(0.3      ,     0.3     ,     -0.1);
      addOffset(0.3      ,     0.3     ,     -0.2);
      addOffset(0.3      ,     0.3     ,      0.1);
      addOffset(0.3      ,     0.3     ,      0.2);
   }


}

