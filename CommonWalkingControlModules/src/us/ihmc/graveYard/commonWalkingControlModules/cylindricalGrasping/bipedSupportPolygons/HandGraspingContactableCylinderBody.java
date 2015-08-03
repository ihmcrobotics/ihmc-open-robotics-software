package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.bipedSupportPolygons;

import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class HandGraspingContactableCylinderBody implements ContactableCylinderBody
{

   private final RigidBody rigidBody;
   private final ReferenceFrame cylinderFrame;
   private final double halfHandWidth;
   private final double cylinderRadius;
   private final double gripStrength;
   private final double gripWeaknessFactor;

   public HandGraspingContactableCylinderBody(RigidBody rigidBody, ReferenceFrame cylinderFrame, double cylinderRadius, double halfHandWidth, double gripStrength, double gripWeaknessFactor)
   {
      this.rigidBody = rigidBody;
      this.cylinderFrame = cylinderFrame;
      this.cylinderRadius = cylinderRadius;
      this.halfHandWidth = halfHandWidth;
      this.gripStrength = gripStrength;
      this.gripWeaknessFactor = gripWeaknessFactor;
   }
   
   public String getName()
   {
      return rigidBody.getName();
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public ReferenceFrame getBodyFrame()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   public ReferenceFrame getCylinderFrame()
   {
      return cylinderFrame;
   }

   public double getHalfHandWidth()
   {
      return halfHandWidth;
   }

   public double getCylinderRadius()
   {
      return cylinderRadius;
   }

   public double getGripStrength()
   {
      return gripStrength;
   }

   public double getGripWeaknessFactor()
   {
      return gripWeaknessFactor;
   }
}
