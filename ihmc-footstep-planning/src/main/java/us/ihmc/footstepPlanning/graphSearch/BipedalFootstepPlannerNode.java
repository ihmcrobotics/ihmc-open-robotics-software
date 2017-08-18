package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class BipedalFootstepPlannerNode
{
   public static final double gridSizeX = 0.05;
   public static final double gridSizeY = 0.05;
   public static final double gridSizeYaw = Math.PI / 18.0;

   private RobotSide footstepSide;
   private RigidBodyTransform soleTransform = new RigidBodyTransform();
   private BipedalFootstepPlannerNode parentNode;

   private ArrayList<BipedalFootstepPlannerNode> childrenNodes = new ArrayList<>();
   private double estimatedCostToGoal;

   private boolean isAtGoal = false;
   private boolean isDead = false;

   private double singleStepCost;
   private double percentageOfFoothold = 1.0;
   private ConvexPolygon2D partialFootholdPolygon;

   public BipedalFootstepPlannerNode(RobotSide footstepSide, RigidBodyTransform soleTransform)
   {
      this.footstepSide = footstepSide;
      this.soleTransform.set(soleTransform);
   }

   public BipedalFootstepPlannerNode(BipedalFootstepPlannerNode nodeToCopy)
   {
      this(nodeToCopy.footstepSide, nodeToCopy.soleTransform);
   }

   public double getSingleStepCost()
   {
      return singleStepCost;
   }

   public void setSingleStepCost(double singleStepCost)
   {
      this.singleStepCost = singleStepCost;
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

   public void setSoleTransform(RigidBodyTransform soleTransform)
   {
      this.soleTransform.set(soleTransform);
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

   public void setToDead()
   {
      isDead = true;
   }

   public boolean isDead()
   {
      return isDead;
   }

   public void getChildren(ArrayList<BipedalFootstepPlannerNode> childrenNodesToPack)
   {
      childrenNodesToPack.addAll(childrenNodes);
   }

   public double getEstimatedCostToGoal()
   {
      return estimatedCostToGoal;
   }

   public void setEstimatedCostToGoal(double estimatedCostToGoal)
   {
      this.estimatedCostToGoal = estimatedCostToGoal;
   }

   public boolean epsilonEquals(BipedalFootstepPlannerNode nodeToCheck, double epsilon)
   {
      if (nodeToCheck.footstepSide != this.footstepSide) return false;
      if (!nodeToCheck.soleTransform.epsilonEquals(this.soleTransform, epsilon)) return false;

      return true;
   }

   private final Vector3D tempPointA = new Vector3D();
   private final Vector3D tempPointB = new Vector3D();
   private final Vector3D tempRotationVectorA = new Vector3D();
   private final Vector3D tempRotationVectorB = new Vector3D();

   @Override
   public boolean equals(Object o)
   {
      if (!(o instanceof BipedalFootstepPlannerNode))
      {
         return false;
      }
      else
      {
         BipedalFootstepPlannerNode otherNode = (BipedalFootstepPlannerNode) o;

         if (getRobotSide() != otherNode.getRobotSide())
         {
            return false;
         }

         this.soleTransform.getTranslation(tempPointA);
         EuclidCoreMissingTools.roundToGivenPrecision(tempPointA, gridSizeX);

         otherNode.soleTransform.getTranslation(tempPointB);
         EuclidCoreMissingTools.roundToGivenPrecision(tempPointB, gridSizeX);

         tempPointA.sub(tempPointB);
         tempPointA.setZ(0.0);

         if (!(tempPointA.length() < 1e-10))
            return false;

         this.soleTransform.getRotationEuler(tempRotationVectorA);
         double thisYaw = AngleTools.roundToGivenPrecisionForAngle(tempRotationVectorA.getZ(), gridSizeYaw);

         otherNode.soleTransform.getRotationEuler(tempRotationVectorB);
         double otherYaw = AngleTools.roundToGivenPrecisionForAngle(tempRotationVectorB.getZ(), gridSizeYaw);

         //         tempRotationVectorA.sub(tempRotationVectorB);
         double yawDifference = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(thisYaw, otherYaw));
         if (!(yawDifference < 1e-10))
            return false;

         return true;
      }
   }

   @Override
   public int hashCode()
   {
      this.soleTransform.getTranslation(tempPointA);
      EuclidCoreMissingTools.roundToGivenPrecision(tempPointA, gridSizeX);

      this.soleTransform.getRotationEuler(tempRotationVectorA);
      AngleTools.roundToGivenPrecisionForAngles(tempRotationVectorA, gridSizeYaw);

      int result = getRobotSide() == null ? 0 : getRobotSide().hashCode();
      result = 3 * result + (int) Math.round(tempPointA.getX() / gridSizeX);
      result = 3 * result + (int) Math.round(tempPointA.getY() / gridSizeX);
      //      result = 3 * result + (int) Math.round(tempRotationVectorA.getX() / YAW_ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

      return result;
   }

   public String toString()
   {
      return soleTransform.toString();
   }

   public boolean isPartialFoothold()
   {
      return MathTools.isLessThanWithPrecision(percentageOfFoothold, 1.0, 1e-3);
   }

   public double getPercentageOfFoothold()
   {
      return percentageOfFoothold;
   }

   public void setPercentageOfFoothold(double percentageOfFoothold)
   {
      this.percentageOfFoothold = percentageOfFoothold;
   }

   public ConvexPolygon2D getPartialFootholdPolygon()
   {
      return partialFootholdPolygon;
   }

   public void setPartialFootholdPolygon(ConvexPolygon2D partialFootholdPolygon)
   {
      this.partialFootholdPolygon = partialFootholdPolygon;
   }
}
