package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceEstimates
{
   private final FrameOrientation bodyOrientation;
   private final FramePoint bodyPosition;
   private final FrameVector bodyLinearVelocity;
   private final FrameVector bodyAngularVelocity;
   private final FramePoint comPosition;
   private final FrameVector comVelocity;
   private final FramePoint dcmPosition;
   private final FramePoint icpPosition;
   private double lipNaturalFrequency;
   private final QuadrantDependentList<FrameOrientation> soleOrientation;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<FrameVector> soleAngularVelocity;
   private final QuadrantDependentList<FrameVector> soleLinearVelocity;

   public QuadrupedTaskSpaceEstimates()
   {
      bodyOrientation = new FrameOrientation();
      bodyPosition = new FramePoint();
      bodyLinearVelocity = new FrameVector();
      bodyAngularVelocity = new FrameVector();
      comPosition = new FramePoint();
      comVelocity = new FrameVector();
      dcmPosition = new FramePoint();
      icpPosition = new FramePoint();
      soleOrientation = new QuadrantDependentList<>();
      solePosition = new QuadrantDependentList<>();
      soleAngularVelocity = new QuadrantDependentList<>();
      soleLinearVelocity = new QuadrantDependentList<>();
      lipNaturalFrequency = Math.sqrt(9.81 / 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleOrientation.set(robotQuadrant, new FrameOrientation());
         solePosition.set(robotQuadrant, new FramePoint());
         soleAngularVelocity.set(robotQuadrant, new FrameVector());
         soleLinearVelocity.set(robotQuadrant, new FrameVector());
      }
   }

   public FrameOrientation getBodyOrientation()
   {
      return bodyOrientation;
   }

   public FramePoint getBodyPosition()
   {
      return bodyPosition;
   }

   public FrameVector getBodyLinearVelocity()
   {
      return bodyLinearVelocity;
   }

   public FrameVector getBodyAngularVelocity()
   {
      return bodyAngularVelocity;
   }

   public FramePoint getComPosition()
   {
      return comPosition;
   }

   public FrameVector getComVelocity()
   {
      return comVelocity;
   }

   public FramePoint getDcmPosition()
   {
      return dcmPosition;
   }

   public FramePoint getIcpPosition()
   {
      return icpPosition;
   }

   public double getLipNaturalFrequency()
   {
      return lipNaturalFrequency;
   }

   public void setLipNaturalFrequency(double lipNaturalFrequency)
   {
      this.lipNaturalFrequency = lipNaturalFrequency;
   }

   public FrameOrientation getSoleOrientation(RobotQuadrant robotQuadrant)
   {
      return soleOrientation.get(robotQuadrant);
   }

   public FramePoint getSolePosition(RobotQuadrant robotQuadrant)
   {
      return solePosition.get(robotQuadrant);
   }

   public FrameVector getSoleAngularVelocity(RobotQuadrant robotQuadrant)
   {
      return soleAngularVelocity.get(robotQuadrant);
   }

   public FrameVector getSoleLinearVelocity(RobotQuadrant robotQuadrant)
   {
      return soleLinearVelocity.get(robotQuadrant);
   }

   public QuadrantDependentList<FrameOrientation> getSoleOrientation()
   {
      return soleOrientation;
   }

   public QuadrantDependentList<FramePoint> getSolePosition()
   {
      return solePosition;
   }

   public QuadrantDependentList<FrameVector> getSoleAngularVelocity()
   {
      return soleAngularVelocity;
   }

   public QuadrantDependentList<FrameVector> getSoleLinearVelocity()
   {
      return soleLinearVelocity;
   }
}
