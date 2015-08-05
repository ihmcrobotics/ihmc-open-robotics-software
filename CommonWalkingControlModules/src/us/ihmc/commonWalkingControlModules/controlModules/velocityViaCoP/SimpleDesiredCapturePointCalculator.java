package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;


public class SimpleDesiredCapturePointCalculator implements DesiredCapturePointCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredCapturePointCalculator");
   private final CouplingRegistry couplingRegistry;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final double controlDT;
   private final double footForward;
   
   enum MotionType
   {
      OFFSET_SUPPORT_POLYGON, CIRCLE, LINE
   }
   public static boolean USEUPCOMINGSWINGTOEASSWEETSPOT = false; //TODO: Get rid of this hack

   private final EnumYoVariable<MotionType> motionType = new EnumYoVariable<SimpleDesiredCapturePointCalculator.MotionType>("iCPMotionType", registry, MotionType.class);
   
   private final DoubleYoVariable desiredCaptureForwardStayInDoubleSupport = new DoubleYoVariable("desiredCaptureForwardNotLoading", registry);
   private final DoubleYoVariable desiredCaptureKxx = new DoubleYoVariable("desiredCaptureKxx", registry);
   private final DoubleYoVariable desiredCaptureKxy = new DoubleYoVariable("desiredCaptureKxy", registry);
   
   private final BooleanYoVariable doICPMotion = new BooleanYoVariable("doICPEdgeTraceMotion", registry);
   private final DoubleYoVariable icpMotionSpeed = new DoubleYoVariable("icpMotionSpeed", registry);
   private final DoubleYoVariable icpMotionDistanceToOuterEdge = new DoubleYoVariable("icpMotionDistanceToOuterEdge", registry);
   private final DoubleYoVariable icpMotionDistanceToOuterEdgeBackFootScaling = new DoubleYoVariable("icpMotionDistanceToOuterEdgeBackFootScaling", registry);
   
   private final DoubleYoVariable icpMotionXYScaling = new DoubleYoVariable("icpMotionXYScaling", registry);
   
   
   
   private final DoubleYoVariable icpCurrentPositionOnMotionPolygon = new DoubleYoVariable("icpCurrentPositionOnMotionPolygon", registry);

   public SimpleDesiredCapturePointCalculator(CouplingRegistry couplingRegistry, CommonHumanoidReferenceFrames referenceFrames, double footForward, double controlDT,
           YoVariableRegistry parentRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      this.referenceFrames = referenceFrames;
      this.controlDT = controlDT;
      this.footForward = footForward * 0.7;
      parentRegistry.addChild(registry);
      
      icpMotionSpeed.set(20.0);
      icpCurrentPositionOnMotionPolygon.set(25.0);
      icpMotionXYScaling.set(1.5);
      motionType.set(MotionType.LINE);
      icpMotionDistanceToOuterEdge.set(0.1); // 0.05
      icpMotionDistanceToOuterEdgeBackFootScaling.set(0.5);
   }

   public FramePoint2d computeDesiredCapturePointSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity, SingleSupportCondition singleSupportCondition)
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

      FramePoint2d desiredCapturePoint = couplingRegistry.getOldBipedSupportPolygons().getSweetSpotCopy(supportLeg);
      ReferenceFrame ankleZUpFrame = referenceFrames.getAnkleZUpFrame(supportLeg);
      desiredCapturePoint.changeFrame(ankleZUpFrame);
      desiredVelocity.changeFrame(ankleZUpFrame);

      desiredCapturePoint.setX(desiredCapturePoint.getX() + kxx * desiredVelocity.getX());
      desiredCapturePoint.setY(desiredCapturePoint.getY() + kxy * Math.abs(desiredVelocity.getX()));
      couplingRegistry.setDesiredCapturePoint(desiredCapturePoint);
      return desiredCapturePoint;
   }
   
   private boolean useToePoint()
   {
      RobotSide upcomingSupportLeg = couplingRegistry.getUpcomingSupportLeg();
      
      if (upcomingSupportLeg == null)
         return false;
      else
      {         
         RobotSide upcomingSwingLeg = upcomingSupportLeg.getOppositeSide();
         
         FramePoint upcomingSupportPoint = new FramePoint(referenceFrames.getAnkleZUpFrame(upcomingSupportLeg));
         upcomingSupportPoint.changeFrame(referenceFrames.getAnkleZUpFrame(upcomingSwingLeg));
         
         return upcomingSupportPoint.getX() > footForward && USEUPCOMINGSWINGTOEASSWEETSPOT; 
      }
   }
   
   private final ConvexPolygonShrinker shrinker = new ConvexPolygonShrinker(); 
   private final FrameConvexPolygon2d motionPolygon = new FrameConvexPolygon2d();

   private FramePoint2d computeCapturePointMotion()
   {
      
      OldBipedSupportPolygons bipedSupportPolygons = couplingRegistry.getOldBipedSupportPolygons();
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      icpCurrentPositionOnMotionPolygon.add(icpMotionSpeed.getDoubleValue() * controlDT);
      
      
      switch(motionType.getEnumValue())
      {
      case LINE:
      {
         RobotSide upcomingSupportLeg = couplingRegistry.getUpcomingSupportLeg();
         RobotSide upcomingSwingLeg = upcomingSupportLeg.getOppositeSide();
         
         FramePoint2d pointA = bipedSupportPolygons.getSweetSpotCopy(upcomingSwingLeg);
         FramePoint2d pointB = bipedSupportPolygons.getSweetSpotCopy(upcomingSupportLeg);
         
         if(useToePoint())
         {
            pointA.setX(pointA.getX() + footForward);
         }
         pointA.changeFrame(referenceFrames.getMidFeetZUpFrame());
         pointB.changeFrame(referenceFrames.getMidFeetZUpFrame());
         
         FrameVector2d AtoB = new FrameVector2d(pointA, pointB);
         AtoB.normalize();
         AtoB.scale(icpMotionDistanceToOuterEdge.getDoubleValue() * icpMotionDistanceToOuterEdgeBackFootScaling.getDoubleValue());
         
         FrameVector2d BtoA = new FrameVector2d(pointB, pointA);
         BtoA.normalize();
         BtoA.scale(icpMotionDistanceToOuterEdge.getDoubleValue());
         
         pointA.add(AtoB);
         pointB.add(BtoA);
         
         
         double percentage = icpCurrentPositionOnMotionPolygon.getDoubleValue() % 100;
         FrameVector2d direction;
         
         
         if(percentage < 50.0)
         {
            direction = new FrameVector2d(pointA, pointB);
            direction.scale(percentage/50.0);
            pointA.add(direction);
            return pointA;
         }
         else
         {
            direction = new FrameVector2d(pointB, pointA);
            direction.scale((percentage - 50.0)/50.0);
            pointB.add(direction);
            return pointB;
         }
         
         
      }
      
      case CIRCLE:
      {
         FramePoint2d desiredCapturePoint = supportPolygon.getCentroidCopy();
         FrameLineSegment2d connectingEdge = bipedSupportPolygons.getConnectingEdge1();
         connectingEdge.changeFrame(desiredCapturePoint.getReferenceFrame());
         double toConnectingEdge = connectingEdge.distance(desiredCapturePoint);
         
         FramePoint2d sweetSpotA = bipedSupportPolygons.getSweetSpotCopy(RobotSide.LEFT);
         FramePoint2d sweetSpotB = bipedSupportPolygons.getSweetSpotCopy(RobotSide.RIGHT);
         sweetSpotB.changeFrame(sweetSpotA.getReferenceFrame());
         double footToFoot = sweetSpotA.distance(sweetSpotB);
         
         icpMotionXYScaling.set(footToFoot/(2.0*toConnectingEdge));
         
         double t = (icpCurrentPositionOnMotionPolygon.getDoubleValue() / 100.0) * 2 * Math.PI;
         double x = Math.cos(t) * icpMotionDistanceToOuterEdge.getDoubleValue();
         double y = Math.sin(t) * icpMotionDistanceToOuterEdge.getDoubleValue() * icpMotionXYScaling.getDoubleValue();
         
         FrameVector2d circleVector = new FrameVector2d(desiredCapturePoint.getReferenceFrame(), x, y);
         desiredCapturePoint.add(circleVector);
         return desiredCapturePoint;
         
      }
      case OFFSET_SUPPORT_POLYGON:
      {
         shrinker.shrinkConstantDistanceInto(supportPolygon, icpMotionDistanceToOuterEdge.getDoubleValue(), motionPolygon);

         if (motionPolygon == null)
         {
            // Shrunken to the smallest possible polygon, the desired CP becomes the center of the polygon.
            return supportPolygon.getCentroidCopy();
         }
         double numberOfPoints = motionPolygon.getNumberOfVertices();

         double totalLength = 0;
         for (int i = 0; i < numberOfPoints; i++)
         {
            int nextPoint = i >= (numberOfPoints - 1) ? 0 : i + 1;
            totalLength += motionPolygon.getFrameVertex(i).distance(motionPolygon.getFrameVertex(nextPoint));
         }

         
         double distanceInPolygon = (icpCurrentPositionOnMotionPolygon.getDoubleValue() % 100) * 0.01 * totalLength;

         double lengthPassed = 0;
         for (int i = 0; i < numberOfPoints; i++)
         {
            double distance = motionPolygon.getFrameVertex(i).distance(motionPolygon.getNextFrameVertex(i));
            if ((lengthPassed + distance) > distanceInPolygon)
            {
               FrameVector2d edge = new FrameVector2d(motionPolygon.getFrameVertex(i), motionPolygon.getNextFrameVertex(i));
               edge.normalize();
               edge.scale(distanceInPolygon - lengthPassed);

               FramePoint2d desiredCapturePoint = new FramePoint2d(motionPolygon.getFrameVertex(i));
               desiredCapturePoint.add(edge);

               return desiredCapturePoint;

            }

            lengthPassed += distance;
         }

      }
      }
      throw new RuntimeException("Should not get here.");
   }

   public FramePoint2d computeDesiredCapturePointDoubleSupport(RobotSide loadingLeg, OldBipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity)
   {
      if (stayInDoubleSupport(loadingLeg))
      {
         if (doICPMotion.getBooleanValue())
         {
            return computeCapturePointMotion();
         } else
         {
            FramePoint2d desiredCapturePoint;
            if(useToePoint())
            {
               RobotSide upcomingSupportLeg = couplingRegistry.getUpcomingSupportLeg();
               RobotSide upcomingSwingLeg = upcomingSupportLeg.getOppositeSide();
               
               FramePoint2d pointA = bipedSupportPolygons.getSweetSpotCopy(upcomingSwingLeg);
               FramePoint2d pointB = bipedSupportPolygons.getSweetSpotCopy(upcomingSupportLeg);
               
               if(useToePoint())
               {
                  pointA.setX(pointA.getX() + footForward);
               }
               pointA.changeFrame(referenceFrames.getMidFeetZUpFrame());
               pointB.changeFrame(referenceFrames.getMidFeetZUpFrame());
               
               FrameLineSegment2d pointToPoint = new FrameLineSegment2d(pointA, pointB);
               desiredCapturePoint = pointToPoint.pointBetweenEndPointsGivenParameter(0.5);
               
            }
            else
            {
               desiredCapturePoint = new FramePoint2d(referenceFrames.getMidFeetZUpFrame());
   
               FrameVector2d leftForward = new FrameVector2d(referenceFrames.getAnkleZUpFrame(RobotSide.LEFT), 1.0, 0.0);
               FrameVector2d rightForward = new FrameVector2d(referenceFrames.getAnkleZUpFrame(RobotSide.RIGHT), 1.0, 0.0);
   
               leftForward.changeFrame(desiredCapturePoint.getReferenceFrame());
               rightForward.changeFrame(desiredCapturePoint.getReferenceFrame());
   
               FrameVector2d offset = leftForward;
               offset.add(rightForward);
               offset.normalize();
               offset.scale(desiredCaptureForwardStayInDoubleSupport.getDoubleValue());
               desiredCapturePoint.add(offset);
            }
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
