package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class BipedalFootstepPlannerNode
{
   public static final double gridSizeX = 0.025;
   public static final double gridSizeY = 0.025;
   public static final double gridSizeYaw = Math.toRadians(2.0);

   private final int xIndex;
   private final int yIndex;
   private final int yawIndex;
   private final RobotSide robotSide;

   private BipedalFootstepPlannerNode parentNode;

   private ArrayList<BipedalFootstepPlannerNode> childrenNodes = new ArrayList<>();
   private double estimatedCostToGoal;

   private boolean isAtGoal = false;
   private boolean isDead = false;

   private double singleStepCost;
   private double percentageOfFoothold = 1.0;
   private ConvexPolygon2D partialFootholdPolygon;

   public BipedalFootstepPlannerNode(double x, double y, double yaw, RobotSide robotSide)
   {
      xIndex = (int) Math.round(x / gridSizeX);
      yIndex = (int) Math.round(y / gridSizeY);
      yawIndex = (int) Math.round(AngleTools.trimAngleMinusPiToPi(yaw) / gridSizeYaw);
      this.robotSide = robotSide;
   }

   public BipedalFootstepPlannerNode(BipedalFootstepPlannerNode nodeToCopy)
   {
      this.xIndex = nodeToCopy.xIndex;
      this.yIndex = nodeToCopy.yIndex;
      this.yawIndex = nodeToCopy.yawIndex;
      this.robotSide = nodeToCopy.robotSide;
   }

   public double getX()
   {
      return gridSizeX * xIndex;
   }

   public double getY()
   {
      return gridSizeY * yIndex;
   }

   public double getYaw()
   {
      return gridSizeYaw * yawIndex;
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
      return robotSide;
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
      soleTransformToPack.setRotationYawAndZeroTranslation(getYaw());
      soleTransformToPack.setTranslation(getX(), getY(), 0.0);
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

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((robotSide == null) ? 0 : robotSide.hashCode());
      result = prime * result + xIndex;
      result = prime * result + yIndex;
      result = prime * result + yawIndex;
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      BipedalFootstepPlannerNode other = (BipedalFootstepPlannerNode) obj;
      if (robotSide != other.robotSide)
         return false;
      if (xIndex != other.xIndex)
         return false;
      if (yIndex != other.yIndex)
         return false;
      if (yawIndex != other.yawIndex)
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "Node: x=" + xIndex + ", y=" + yIndex;
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
