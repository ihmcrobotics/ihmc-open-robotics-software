package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SimpleDesiredCapturePointCalculator implements DesiredCapturePointCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredCapturePointCalculator");
   private final CouplingRegistry couplingRegistry;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final double controlDT;
   
   enum MotionType
   {
      OFFSET_SUPPORT_POLYGON, CIRCLE
   }
   
   private final EnumYoVariable<MotionType> motionType = new EnumYoVariable<SimpleDesiredCapturePointCalculator.MotionType>("iCPMotionType", registry, MotionType.class);
   
   private final DoubleYoVariable desiredCaptureForwardStayInDoubleSupport = new DoubleYoVariable("desiredCaptureForwardNotLoading", registry);
   private final DoubleYoVariable desiredCaptureKxx = new DoubleYoVariable("desiredCaptureKxx", registry);
   private final DoubleYoVariable desiredCaptureKxy = new DoubleYoVariable("desiredCaptureKxy", registry);
   
   private final BooleanYoVariable doICPMotion = new BooleanYoVariable("doICPEdgeTraceMotion", registry);
   private final DoubleYoVariable icpMotionSpeed = new DoubleYoVariable("icpMotionSpeed", registry);
   private final DoubleYoVariable icpMotionDistanceToOuterEdge = new DoubleYoVariable("icpMotionDistanceToOuterEdge", registry);
   private final DoubleYoVariable icpMotionXYScaling = new DoubleYoVariable("icpMotionXYScaling", registry);
   
   private final DoubleYoVariable icpCurrentPositionOnMotionPolygon = new DoubleYoVariable("icpCurrentPositionOnMotionPolygon", registry);

   public SimpleDesiredCapturePointCalculator(CouplingRegistry couplingRegistry, CommonWalkingReferenceFrames referenceFrames, double controlDT,
           YoVariableRegistry parentRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      this.referenceFrames = referenceFrames;
      this.controlDT = controlDT;
      parentRegistry.addChild(registry);
      
      icpMotionSpeed.set(10.0);
      icpMotionDistanceToOuterEdge.set(0.05);
      icpCurrentPositionOnMotionPolygon.set(0.0);
      icpMotionXYScaling.set(1.5);
      motionType.set(MotionType.CIRCLE);
   }

   public FramePoint2d computeDesiredCapturePointSingleSupport(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity, SingleSupportCondition singleSupportCondition)
   { 
//      return couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(supportLeg);      
      double kxx, kxy;

//      if (singleSupportCondition == SingleSupportCondition.Loading)
//      {
//         kxx = 0.0;
//         kxy = 0.0;
//      }
//      else
//      {
//         kxx = desiredCaptureKxx.getDoubleValue();
//         kxy = supportLeg.negateIfLeftSide(desiredCaptureKxy.getDoubleValue());
//      }
      kxx = desiredCaptureKxx.getDoubleValue();
      kxy = supportLeg.negateIfLeftSide(desiredCaptureKxy.getDoubleValue());

      FramePoint2d desiredCapturePoint = couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(supportLeg);
      ReferenceFrame ankleZUpFrame = referenceFrames.getAnkleZUpFrame(supportLeg);
      desiredCapturePoint.changeFrame(ankleZUpFrame);
      desiredVelocity = desiredVelocity.changeFrameCopy(ankleZUpFrame);

      desiredCapturePoint.setX(desiredCapturePoint.getX() + kxx * desiredVelocity.getX());
      desiredCapturePoint.setY(desiredCapturePoint.getY() + kxy * Math.abs(desiredVelocity.getX()));
      couplingRegistry.setDesiredCapturePoint(desiredCapturePoint);
      return desiredCapturePoint;
   }
   
   private FramePoint2d computeCapturePointMotion()
   {
      
      FrameConvexPolygon2d supportPolygon = couplingRegistry.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
      icpCurrentPositionOnMotionPolygon.add(icpMotionSpeed.getDoubleValue() * controlDT);
      
      
      switch(motionType.getEnumValue())
      {
      case CIRCLE:
      {
         FramePoint2d desiredCapturePoint = supportPolygon.getCentroidCopy();
         
         double t = (icpCurrentPositionOnMotionPolygon.getDoubleValue() / 100.0) * 2 * Math.PI;
         double x = Math.cos(t) * icpMotionDistanceToOuterEdge.getDoubleValue();
         double y = Math.sin(t) * icpMotionDistanceToOuterEdge.getDoubleValue() * icpMotionXYScaling.getDoubleValue();
         
         FrameVector2d circleVector = new FrameVector2d(desiredCapturePoint.getReferenceFrame(), x, y);
         desiredCapturePoint.add(circleVector);
         return desiredCapturePoint;
         
      }
      case OFFSET_SUPPORT_POLYGON:
      {
         FrameConvexPolygon2d motionPolygon = FrameConvexPolygon2d.shrinkConstantDistanceInto(icpMotionDistanceToOuterEdge.getDoubleValue(), supportPolygon);

         if (motionPolygon == null)
         {
            // Shrunken to the smallest possible polygon, the desired CP becomes the center of the polygon.
            return supportPolygon.getCentroidCopy();
         }
         ArrayList<FramePoint2d> pointsOnPolygon = motionPolygon.getClockwiseOrderedListOfFramePoints();

         double numberOfPoints = pointsOnPolygon.size();

         double totalLength = 0;
         for (int i = 0; i < numberOfPoints; i++)
         {
            int nextPoint = i >= (numberOfPoints - 1) ? 0 : i + 1;
            totalLength += pointsOnPolygon.get(i).distance(pointsOnPolygon.get(nextPoint));
         }

         
         double distanceInPolygon = (icpCurrentPositionOnMotionPolygon.getDoubleValue() % 100) * 0.01 * totalLength;

         double lengthPassed = 0;
         for (int i = 0; i < numberOfPoints; i++)
         {
            int nextPoint = i >= (numberOfPoints - 1) ? 0 : i + 1;
            double distance = pointsOnPolygon.get(i).distance(pointsOnPolygon.get(nextPoint));
            if ((lengthPassed + distance) > distanceInPolygon)
            {
               FrameVector2d edge = new FrameVector2d(pointsOnPolygon.get(i), pointsOnPolygon.get(nextPoint));
               edge.normalize();
               edge.scale(distanceInPolygon - lengthPassed);

               FramePoint2d desiredCapturePoint = new FramePoint2d(pointsOnPolygon.get(i));
               desiredCapturePoint.add(edge);

               return desiredCapturePoint;

            }

            lengthPassed += distance;
         }

      }
      }
      throw new RuntimeException("Should not get here.");
   }

   public FramePoint2d computeDesiredCapturePointDoubleSupport(RobotSide loadingLeg, BipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity)
   {
      if (stayInDoubleSupport(loadingLeg))
      {
         if (doICPMotion.getBooleanValue())
         {
            return computeCapturePointMotion();
         } else
         {
            FramePoint2d desiredCapturePoint = new FramePoint2d(referenceFrames.getMidFeetZUpFrame());

            FrameVector2d leftForward = new FrameVector2d(referenceFrames.getAnkleZUpFrame(RobotSide.LEFT), 1.0, 0.0);
            FrameVector2d rightForward = new FrameVector2d(referenceFrames.getAnkleZUpFrame(RobotSide.RIGHT), 1.0, 0.0);

            leftForward.changeFrame(desiredCapturePoint.getReferenceFrame());
            rightForward.changeFrame(desiredCapturePoint.getReferenceFrame());

            FrameVector2d offset = leftForward;
            offset.add(rightForward);
            offset.normalize();
            offset.scale(desiredCaptureForwardStayInDoubleSupport.getDoubleValue());
            desiredCapturePoint.add(offset);
            couplingRegistry.setDesiredCapturePoint(desiredCapturePoint);
            return desiredCapturePoint;
         }
      }
      else
      {
         return computeDesiredCapturePointSingleSupport(loadingLeg, bipedSupportPolygons, desiredVelocity, SingleSupportCondition.Loading);
      }
   }

   public void setUpParametersForR2()
   {
      desiredCaptureForwardStayInDoubleSupport.set(0.05);
      desiredCaptureKxx.set(0.15);
      desiredCaptureKxy.set(0.05);
   }

   public void setUpParametersForM2V2()
   {
      desiredCaptureForwardStayInDoubleSupport.set(0.02);
      desiredCaptureKxx.set(0.03);
      desiredCaptureKxy.set(0.01);
   }
   
   private static boolean stayInDoubleSupport(RobotSide loadingLeg)
   {
      return loadingLeg == null;
   }
}
