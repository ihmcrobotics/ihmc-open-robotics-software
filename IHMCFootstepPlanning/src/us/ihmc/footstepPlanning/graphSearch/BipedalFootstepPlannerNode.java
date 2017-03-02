package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class BipedalFootstepPlannerNode
{
   private RobotSide footstepSide;
   private RigidBodyTransform soleTransform = new RigidBodyTransform();
   private BipedalFootstepPlannerNode parentNode;

   private ArrayList<BipedalFootstepPlannerNode> childrenNodes = new ArrayList<>();
   private double estimatedCostToGoal;

   private static final double XY_DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL = 0.05;
   private static final double YAW_ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL = Math.toRadians(5.0);

   private boolean isAtGoal = false;
   private boolean isDead = false;

   private double singleStepCost;
   private double percentageOfFoothold = 1.0;
   private ConvexPolygon2d partialFootholdPolygon;

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

   public RigidBodyTransform getTransformToParentCopy()
   {
      if (parentNode == null)
         return null;

      RigidBodyTransform transformToParent = new RigidBodyTransform();

      getTransformToParent(transformToParent);
      return transformToParent;
   }
   
   public void getTransformToParent(RigidBodyTransform transformToParentToPack)
   {
      if (parentNode == null)
      {
         transformToParentToPack.setIdentity();
      }
      
      parentNode.getSoleTransform(transformToParentToPack);
      transformToParentToPack.invert();

      transformToParentToPack.multiply(soleTransform);
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

   public Point3D getSolePosition()
   {
      Point3D currentSolePosition = new Point3D();
      soleTransform.transform(currentSolePosition);
      return currentSolePosition;
   }

   public double getSoleYaw()
   {
      Vector3D eulerAngles = new Vector3D();
      soleTransform.getRotationEuler(eulerAngles);
      return eulerAngles.getZ();
   }

   public void transformSoleTransformWithSnapTransformFromZeroZ(RigidBodyTransform snapTransform, PlanarRegion planarRegion)
   {
      // Ignore the z since the snap transform snapped from z = 0. Keep everything else.
      soleTransform.setTranslationZ(0.0);
      soleTransform.preMultiply(snapTransform);
   }

   public void shiftInSoleFrame(Vector2D shiftVector)
   {
      RigidBodyTransform shiftTransform = new RigidBodyTransform();
      shiftTransform.setTranslation(new Vector3D(shiftVector.getX(), shiftVector.getY(), 0.0));
      soleTransform.multiply(shiftTransform);
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

   public double getCostToHereFromStart()
   {
      if (parentNode == null)
         return getSingleStepCost();
      return getSingleStepCost() + parentNode.getCostToHereFromStart();
   }

   public double getEstimatedCostToGoal()
   {
      return estimatedCostToGoal;
   }

   public void setEstimatedCostToGoal(double estimatedCostToGoal)
   {
      this.estimatedCostToGoal = estimatedCostToGoal;
   }

   private final Vector3D tempPointA = new Vector3D();
   private final Vector3D tempPointB = new Vector3D();
   private final Vector3D tempRotationVectorA = new Vector3D();
   private final Vector3D tempRotationVectorB = new Vector3D();

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
         EuclidCoreMissingTools.roundToGivenPrecision(tempPointA, XY_DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

         otherNode.soleTransform.getTranslation(tempPointB);
         EuclidCoreMissingTools.roundToGivenPrecision(tempPointB, XY_DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

         tempPointA.sub(tempPointB);
         tempPointA.setZ(0.0);

         if (!(tempPointA.length() < 1e-10)) return false;


         this.soleTransform.getRotationEuler(tempRotationVectorA);
         double thisYaw = AngleTools.roundToGivenPrecisionForAngle(tempRotationVectorA.getZ(), YAW_ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

         otherNode.soleTransform.getRotationEuler(tempRotationVectorB);
         double otherYaw = AngleTools.roundToGivenPrecisionForAngle(tempRotationVectorB.getZ(), YAW_ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);


//         tempRotationVectorA.sub(tempRotationVectorB);
         double yawDifference = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(thisYaw, otherYaw));
         if (!(yawDifference < 1e-10)) return false;

         return true;
      }
   }

   @Override
   public int hashCode()
   {
      this.soleTransform.getTranslation(tempPointA);
      EuclidCoreMissingTools.roundToGivenPrecision(tempPointA, XY_DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

      this.soleTransform.getRotationEuler(tempRotationVectorA);
      AngleTools.roundToGivenPrecisionForAngles(tempRotationVectorA, YAW_ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

      int result = getRobotSide() == null ? 0 : getRobotSide().hashCode();
      result = 3 * result + (int) Math.round(tempPointA.getX() / XY_DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);
      result = 3 * result + (int) Math.round(tempPointA.getY() / XY_DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL);
//      result = 3 * result + (int) Math.round(tempRotationVectorA.getX() / YAW_ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL);

      return result;
   }

   public static double getXyDistanceThresholdToConsiderNodesEqual()
   {
      return XY_DISTANCE_THRESHOLD_TO_CONSIDER_NODES_EQUAL;
   }

   public static double getYawRotationThresholdToConsiderNodesEqual()
   {
      return YAW_ROTATION_THRESHOLD_TO_CONSIDER_NODES_EQUAL;
   }

   public boolean epsilonEquals(BipedalFootstepPlannerNode nodeToCheck, double epsilon)
   {
      if (nodeToCheck.footstepSide != this.footstepSide) return false;
      if (!nodeToCheck.soleTransform.epsilonEquals(this.soleTransform, epsilon)) return false;

      return true;
   }

   public String toString()
   {
      return soleTransform.toString();
   }

   public boolean isPartialFoothold()
   {
      return MathTools.isPreciselyLessThan(percentageOfFoothold, 1.0, 1e-3);
   }

   public double getPercentageOfFoothold()
   {
      return percentageOfFoothold;
   }

   public void setPercentageOfFoothold(double percentageOfFoothold)
   {
      this.percentageOfFoothold = percentageOfFoothold;
   }


   public ConvexPolygon2d getPartialFootholdPolygon()
   {
      return partialFootholdPolygon;
   }

   public void setPartialFootholdPolygon(ConvexPolygon2d partialFootholdPolygon)
   {
      this.partialFootholdPolygon = partialFootholdPolygon;
   }

   public void removePitchAndRoll()
   {
      RotationTools.removePitchAndRollFromTransform(soleTransform);
   }
}
