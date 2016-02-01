package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


// Class defining a contactable rolling body
public class ListofPointsContactableRollingBody implements ContactableRollingBody
{
   private final RigidBody rigidBody;
   private final double cylinderRadius;
   // Distance along the x-axis between the contact point and the originInBodyFrame
   private final List<Double> contactPoints = new ArrayList<Double>();
   private final ReferenceFrame bodyFrame;
   // Origin of the cylinder in the body frame
   private final FramePoint originInBodyFrame;
   private final String name;
   private final int totalNumberOfContactPoints;

   public ListofPointsContactableRollingBody(String name, RigidBody rigidBody, ReferenceFrame bodyFrame, double cylinderRadius, FramePoint originInBodyFrame,
         List<Double> contactPositionOnCylinderEdge, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      this.rigidBody = rigidBody;
      this.bodyFrame = bodyFrame;
      this.cylinderRadius = cylinderRadius;
      this.contactPoints.addAll(contactPositionOnCylinderEdge);
      this.name = name;

      if (originInBodyFrame.getReferenceFrame() != bodyFrame)
      {
         throw new ReferenceFrameMismatchException("ListofPointsContactableCylinderBody: The position frame (" + originInBodyFrame.getReferenceFrame()
               + ") does not match the body frame (" + bodyFrame + ")");
      }
      this.originInBodyFrame = originInBodyFrame;
      
      totalNumberOfContactPoints = contactPoints.size();
   }

   public String getName()
   {
      return name;
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   //TODO: This is wrong but it works for now... Should be refactored
   public List<FramePoint> getContactPointsCopy()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(contactPoints.size());
      for (int i = 0; i < contactPoints.size(); i++)
      {
         ret.add(new FramePoint(bodyFrame, contactPoints.get(i), 0.0, 0.0));
      }

      return ret;
   }

   public ReferenceFrame getFrameAfterParentJoint()
   {
      return bodyFrame;
   }

   //TODO: This is wrong but it works for now... Should be refactored
   public ReferenceFrame getSoleFrame()
   {
      return bodyFrame;
   }

   //TODO: This is wrong but it works for now... Should be refactored
   public List<FramePoint2d> getContactPoints2d()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(contactPoints.size());
      for (int i = 0; i < contactPoints.size(); i++)
      {
         ret.add(new FramePoint2d(bodyFrame, new Point2d(contactPoints.get(i), 0)));
      }

      return ret;
   }

   public FramePoint getCylinderOriginCopy()
   {
      return new FramePoint(originInBodyFrame);
   }

   public double getCylinderRadius()
   {
      return cylinderRadius;
   }

   public int getTotalNumberOfContactPoints()
   {
      return totalNumberOfContactPoints;
   }
}
