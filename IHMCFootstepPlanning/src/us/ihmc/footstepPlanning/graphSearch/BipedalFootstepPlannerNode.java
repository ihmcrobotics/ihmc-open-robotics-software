package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class BipedalFootstepPlannerNode
{
   private RobotSide footstepSide;
   private RigidBodyTransform soleTransform = new RigidBodyTransform();
   private BipedalFootstepPlannerNode parentNode;

   private ArrayList<BipedalFootstepPlannerNode> childrenNodes;
   private double costFromParent;
   private double costToHereFromStart;
   private double estimatedCostToGoal;

   private static final double DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL = 0.2;
   private static final double ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL = 0.2;

   private boolean isAtGoal = false;

   public BipedalFootstepPlannerNode(RobotSide footstepSide, RigidBodyTransform soleTransform)
   {
      this.footstepSide = footstepSide;
      this.soleTransform.set(soleTransform);
   }

   public BipedalFootstepPlannerNode(BipedalFootstepPlannerNode nodeToCopy)
   {
      this(nodeToCopy.footstepSide, nodeToCopy.soleTransform);
   }

   public RigidBodyTransform getTransformToParent()
   {
      if (parentNode == null)
         return null;

      RigidBodyTransform transformToParent = new RigidBodyTransform();

      parentNode.getSoleTransform(transformToParent);
      transformToParent.invert();

      transformToParent.multiply(transformToParent, soleTransform);
      return transformToParent;
   }

   public RobotSide getRobotSide()
   {
      return footstepSide;
   }

   public void setIsAtGoal()
   {
      this.isAtGoal = true;
   }

   public boolean isAtGoal()
   {
      return isAtGoal;
   }

   public void getSoleTransform(RigidBodyTransform soleTransformToPack)
   {
      soleTransformToPack.set(soleTransform);
   }

   public Point3d getSolePosition()
   {
      Point3d currentSolePosition = new Point3d();
      soleTransform.transform(currentSolePosition);
      return currentSolePosition;
   }

   public double getSoleYaw()
   {
      Vector3d eulerAngles = new Vector3d();
      soleTransform.getRotationEuler(eulerAngles);
      return eulerAngles.getZ();
   }

   public void transformSoleTransformWithSnapTransformFromZeroZ(RigidBodyTransform snapTransform, PlanarRegion planarRegion)
   {
      // Ignore the z since the snap transform snapped from z = 0. Keep everything else.
      soleTransform.setM23(0.0);
      soleTransform.multiply(snapTransform, soleTransform);
   }

   public BipedalFootstepPlannerNode getParentNode()
   {
      return parentNode;
   }

   public void setParentNode(BipedalFootstepPlannerNode parentNode)
   {
      this.parentNode = parentNode;
   }

   public void addChild(BipedalFootstepPlannerNode childNode)
   {
      if (childrenNodes == null)
      {
         childrenNodes = new ArrayList<>();
      }

      this.childrenNodes.add(childNode);
   }

   public void getChildren(ArrayList<BipedalFootstepPlannerNode> childrenNodesToPack)
   {
      childrenNodesToPack.addAll(childrenNodes);
   }

   public double getCostFromParent()
   {
      return costFromParent;
   }

   public void setCostFromParent(double costFromParent)
   {
      this.costFromParent = costFromParent;
   }

   public double getCostToHereFromStart()
   {
      return costToHereFromStart;
   }

   public void setCostToHereFromStart(double costToHereFromStart)
   {
      this.costToHereFromStart = costToHereFromStart;
   }

   public double getEstimatedCostToGoal()
   {
      return estimatedCostToGoal;
   }

   public void setEstimatedCostToGoal(double estimatedCostToGoal)
   {
      this.estimatedCostToGoal = estimatedCostToGoal;
   }

   private final Vector3d tempPointA = new Vector3d();
   private final Vector3d tempPointB = new Vector3d();
   private final Vector3d tempRotationVectorA = new Vector3d();
   private final Vector3d tempRotationVectorB = new Vector3d();

   @Override
   public boolean equals(Object o)
   {
      if(!(o instanceof BipedalFootstepPlannerNode))
      {
         return false;
      }
      else
      {
         BipedalFootstepPlannerNode otherNode = (BipedalFootstepPlannerNode) o;

         if(getRobotSide() != otherNode.getRobotSide())
         {
            return false;
         }

         this.soleTransform.getTranslation(tempPointA);
         MathTools.roundToGivenPrecision(tempPointA, DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

         otherNode.soleTransform.getTranslation(tempPointB);
         MathTools.roundToGivenPrecision(tempPointB, DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

         this.soleTransform.getRotationEuler(tempRotationVectorA);
         MathTools.roundToGivenPrecisionForAngles(tempRotationVectorA, ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

         otherNode.soleTransform.getRotationEuler(tempRotationVectorB);
         MathTools.roundToGivenPrecisionForAngles(tempRotationVectorB, ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

         tempPointA.sub(tempPointB);
         tempRotationVectorA.sub(tempRotationVectorB);

         return Math.abs(tempPointA.length()) < 1e-10 && Math.abs(tempRotationVectorA.length()) < 1e-10;
      }
   }

   @Override
   public int hashCode()
   {
      this.soleTransform.getTranslation(tempPointA);
      MathTools.roundToGivenPrecision(tempPointA, DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

      this.soleTransform.getRotationEuler(tempRotationVectorA);
      MathTools.roundToGivenPrecisionForAngles(tempRotationVectorA, ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

      int result = getRobotSide().hashCode();
      result = 3 * result + (int) Math.round(tempPointA.getX() / DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);
      result = 3 * result + (int) Math.round(tempPointA.getY() / DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);
      result = 3 * result + (int) Math.round(tempPointA.getZ() / DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);
      result = 3 * result + (int) Math.round(tempRotationVectorA.getX() / ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);
      result = 3 * result + (int) Math.round(tempRotationVectorA.getY() / ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);
      result = 3 * result + (int) Math.round(tempRotationVectorA.getZ() / ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

      return result;
   }

   public static double getDistanceThresholdToConsiderNodesEqual()
   {
      return DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL;
   }

   public static double getRotationThresholdToConsiderNodesEqual()
   {
      return ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL;
   }
}
