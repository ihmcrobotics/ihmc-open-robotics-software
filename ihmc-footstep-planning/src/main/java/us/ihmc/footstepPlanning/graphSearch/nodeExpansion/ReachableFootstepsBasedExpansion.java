package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import java.util.HashSet;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
/**
 * This class expands the node based on a fixed number of defined configurations.
 * The list of neighbours generated is robot-specific.
 * @author Shlok Agarwal
 *
 */
public abstract class ReachableFootstepsBasedExpansion implements FootstepNodeExpansion
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private ReferenceFrame stanceFrame;
   private RobotSide stepSide;
   private double yawStanceFoot;
   private HashSet<FootstepNode> neighbors;

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
      
      addNeighbors();

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
    * This function adds footstep neighbors specific to the robot.
    * You can find implementation in AtlasReachableFootstepExpansion & ValkyrieReachableFootstepExpansion
    */
   public abstract void addNeighbors();

}

