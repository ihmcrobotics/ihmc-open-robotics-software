package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceEstimates
{
   private final QuadrantDependentList<FrameOrientation> soleOrientation;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<FrameVector> soleAngularVelocity;
   private final QuadrantDependentList<FrameVector> soleLinearVelocity;
   private final QuadrupedSupportPolygon supportPolygon;
   private final FramePoint supportCentroid;
   private final FrameOrientation supportOrientation;
   private final FrameOrientation bodyOrientation;
   private final FramePoint bodyPosition;
   private final FrameVector bodyLinearVelocity;
   private final FrameVector bodyAngularVelocity;
   private final FramePoint comPosition;
   private final FrameVector comVelocity;
   private final FramePoint dcmPosition;
   private final FramePoint icpPosition;
   private double comHeight;

   public QuadrupedTaskSpaceEstimates()
   {
      soleOrientation = new QuadrantDependentList<>();
      solePosition = new QuadrantDependentList<>();
      soleAngularVelocity = new QuadrantDependentList<>();
      soleLinearVelocity = new QuadrantDependentList<>();
      supportPolygon = new QuadrupedSupportPolygon();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleOrientation.set(robotQuadrant, new FrameOrientation());
         solePosition.set(robotQuadrant, new FramePoint());
         soleAngularVelocity.set(robotQuadrant, new FrameVector());
         soleLinearVelocity.set(robotQuadrant, new FrameVector());
         supportPolygon.setFootstep(robotQuadrant, new FramePoint());
      }
      supportCentroid = new FramePoint();
      supportOrientation = new FrameOrientation();
      bodyOrientation = new FrameOrientation();
      bodyPosition = new FramePoint();
      bodyLinearVelocity = new FrameVector();
      bodyAngularVelocity = new FrameVector();
      comPosition = new FramePoint();
      comVelocity = new FrameVector();
      dcmPosition = new FramePoint();
      icpPosition = new FramePoint();
      comHeight = 1.0;
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

   public QuadrupedSupportPolygon getSupportPolygon()
   {
      return supportPolygon;
   }

   public FramePoint getSupportCentroid()
   {
      return supportCentroid;
   }

   public FrameOrientation getSupportOrientation()
   {
      return supportOrientation;
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

   public double getComHeight()
   {
      return comHeight;
   }

   public void setComHeight(double comHeight)
   {
      this.comHeight = comHeight;
   }
}
