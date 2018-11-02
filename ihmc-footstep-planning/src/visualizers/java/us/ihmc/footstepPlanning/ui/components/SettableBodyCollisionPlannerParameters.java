package us.ihmc.footstepPlanning.ui.components;

import us.ihmc.footstepPlanning.graphSearch.parameters.BodyCollisionPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class SettableBodyCollisionPlannerParameters implements BodyCollisionPlannerParameters
{
      private boolean checkForBodyBoxCollisions;
   private double bodyBoxWidth;
   private double bodyBoxHeight;
   private double bodyBoxDepth;
   private double bodyBoxBaseX;
   private double bodyBoxBaseY;
   private double bodyBoxBaseZ;

   public SettableBodyCollisionPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      set(footstepPlannerParameters);
   }

   public SettableBodyCollisionPlannerParameters(BodyCollisionPlannerParameters footstepPlannerParameters)
   {
      set(footstepPlannerParameters);
   }
   
   public void set(FootstepPlannerParameters footstepPlannerParameters)
   {
      this.checkForBodyBoxCollisions = footstepPlannerParameters.checkForBodyBoxCollisions();
      this.bodyBoxHeight = footstepPlannerParameters.getBodyBoxHeight();
      this.bodyBoxWidth = footstepPlannerParameters.getBodyBoxWidth();
      this.bodyBoxDepth = footstepPlannerParameters.getBodyBoxDepth();
      this.bodyBoxBaseX = footstepPlannerParameters.getBodyBoxBaseX();
      this.bodyBoxBaseY = footstepPlannerParameters.getBodyBoxBaseY();
      this.bodyBoxBaseZ = footstepPlannerParameters.getBodyBoxBaseZ();
   }
   
   public void set(BodyCollisionPlannerParameters footstepPlannerParameters)
   {
      this.checkForBodyBoxCollisions = footstepPlannerParameters.checkForBodyBoxCollisions();
      this.bodyBoxHeight = footstepPlannerParameters.getBodyBoxHeight();
      this.bodyBoxWidth = footstepPlannerParameters.getBodyBoxWidth();
      this.bodyBoxDepth = footstepPlannerParameters.getBodyBoxDepth();
      this.bodyBoxBaseX = footstepPlannerParameters.getBodyBoxBaseX();
      this.bodyBoxBaseY = footstepPlannerParameters.getBodyBoxBaseY();
      this.bodyBoxBaseZ = footstepPlannerParameters.getBodyBoxBaseZ();
   }

   public void setCheckForBodyBoxCollisions(boolean checkForBodyBoxCollisions)
   {
      this.checkForBodyBoxCollisions = checkForBodyBoxCollisions;
   }

   public void setBodyBoxWidth(double bodyBoxWidth)
   {
      this.bodyBoxWidth = bodyBoxWidth;
   }

   public void setBodyBoxHeight(double bodyBoxHeight)
   {
      this.bodyBoxHeight = bodyBoxHeight;
   }

   public void setBodyBoxDepth(double bodyBoxDepth)
   {
      this.bodyBoxDepth = bodyBoxDepth;
   }

   public void setBodyBoxBaseX(double bodyBoxBaseX)
   {
      this.bodyBoxBaseX = bodyBoxBaseX;
   }

   public void setBodyBoxBaseY(double bodyBoxBaseY)
   {
      this.bodyBoxBaseY = bodyBoxBaseY;
   }

   public void setBodyBoxBaseZ(double bodyBoxBaseZ)
   {
      this.bodyBoxBaseZ = bodyBoxBaseZ;
   }

   @Override
   public boolean checkForBodyBoxCollisions()
   {
      return checkForBodyBoxCollisions;
   }

   @Override
   public double getBodyBoxHeight()
   {
      return bodyBoxHeight;
   }

   @Override
   public double getBodyBoxWidth()
   {
      return bodyBoxWidth;
   }

   @Override
   public double getBodyBoxDepth()
   {
      return bodyBoxDepth;
   }

   @Override
   public double getBodyBoxBaseX()
   {
      return bodyBoxBaseX;
   }

   @Override
   public double getBodyBoxBaseY()
   {
      return bodyBoxBaseY;
   }

   @Override
   public double getBodyBoxBaseZ()
   {
      return bodyBoxBaseZ;
   }
}
