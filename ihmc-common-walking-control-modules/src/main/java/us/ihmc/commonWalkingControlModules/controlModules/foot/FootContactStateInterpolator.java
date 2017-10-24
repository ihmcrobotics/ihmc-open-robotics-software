package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameLineSegment;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootContactStateInterpolator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   
   private final List<YoContactPoint> contactPoints;
   private final FrameLineSegment2d contactLine2D = new FrameLineSegment2d();

   private final YoPlaneContactState contactState;
   private final boolean[] contactPointInterpolationActivated;
   private final HashMap<YoContactPoint, FramePoint2D> initialPositions = new HashMap<>();
   private final HashMap<YoContactPoint, FramePoint2D> finalPositions = new HashMap<>();
   
   private final FramePoint2D contactPointPosition = new FramePoint2D();
   
   private final ReferenceFrame planeFrame;
   private final YoDouble alpha;
   private final YoDouble timeInTrajectory;
   private final YoDouble duration;
   private final YoBoolean moveContactPoints;
   private final YoPolynomial polynomial;
   private final double dt;
   

   public FootContactStateInterpolator(RobotSide robotSide, YoPlaneContactState contactState, double dt, YoVariableRegistry parentRegistry)
   {
      this.contactState = contactState;
      this.dt = dt;
      String prefix = robotSide.getCamelCaseNameForStartOfExpression() + name;
      this.registry = new YoVariableRegistry(prefix);
      this.contactPoints = contactState.getContactPoints();
      contactPointInterpolationActivated = new boolean[contactPoints.size()];
      
      planeFrame = contactState.getPlaneFrame();
      
      this.alpha = new YoDouble(prefix + "alpha", registry);
      this.timeInTrajectory = new YoDouble(prefix + "timeInTrajectory", registry);
      this.duration = new YoDouble(prefix + "duration", registry);
      this.moveContactPoints = new YoBoolean("moveContactPoints", registry);
      this.polynomial = new YoPolynomial("rhoWeightTraj", 3, registry);
      
      moveContactPoints.set(false);
      
      for(YoContactPoint yoContactPoint : contactPoints)
      {
         FramePoint2D originalPosition = new FramePoint2D();
         yoContactPoint.getPosition2d(originalPosition);
         
         initialPositions.put(yoContactPoint, new FramePoint2D());
         finalPositions.put(yoContactPoint, originalPosition);
      }
      
      parentRegistry.addChild(registry);
   }
   
   public void initialize(double duration, FrameLineSegment contactLine)
   {
      contactLine.changeFrame(planeFrame);
      contactLine.getIncludingFrame(contactLine2D);
      
      this.duration.set(duration);
      timeInTrajectory.set(0.0);
      
      polynomial.setQuadratic(0.0, duration, 1.0, -0.2,  0.00005);
      
      for(int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint yoContactPoint = contactPoints.get(i);
         if(yoContactPoint.isInContact())
         {
            contactPointInterpolationActivated[i] = false;
         }
         else
         {
            contactPointInterpolationActivated[i] = true;
            yoContactPoint.getPosition2d(contactPointPosition);
            contactPointPosition.changeFrame(planeFrame);
            
            FramePoint2D initialPositionToPack = initialPositions.get(yoContactPoint);
            contactLine2D.orthogonalProjection(contactPointPosition, initialPositionToPack);
         }
      }
   }

   public void update()
   {
      timeInTrajectory.add(dt);
      double time =  MathTools.clamp(timeInTrajectory.getDoubleValue(), 0.0, duration.getDoubleValue()) ;
      polynomial.compute(time);
      
//      alpha.set(timeInTrajectory.getDoubleValue() / duration.getDoubleValue());
      alpha.set(polynomial.getPosition());
      alpha.set(MathTools.clamp(alpha.getDoubleValue(), 0.0, 1.0));
      
      for(int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint yoContactPoint = contactPoints.get(i);
         if(contactPointInterpolationActivated[i])
         {
            if(moveContactPoints.getBooleanValue())
            {
               FramePoint2D initialPosition = initialPositions.get(yoContactPoint);
               FramePoint2D finalPosition = finalPositions.get(yoContactPoint);
               
               contactPointPosition.changeFrame(planeFrame);
               contactPointPosition.interpolate(initialPosition, finalPosition, alpha.getDoubleValue());
               contactPointPosition.changeFrame(yoContactPoint.getReferenceFrame());
               yoContactPoint.setPosition2d(contactPointPosition);
            }
            
            contactState.setRhoWeight(yoContactPoint, alpha.getDoubleValue());
         }
      }
      
//      contactState.notifyContactStateHasChanged();
   }
   
   public boolean isDone()
   {
      return duration.getDoubleValue() - timeInTrajectory.getDoubleValue() <= 0 ;
   }

   public void resetContactState()
   {
      for(int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint yoContactPoint = contactPoints.get(i);
         FramePoint2D finalPosition = finalPositions.get(yoContactPoint);
         contactPointPosition.setIncludingFrame(finalPosition);
         contactPointPosition.changeFrame(yoContactPoint.getReferenceFrame());
         yoContactPoint.setPosition2d(contactPointPosition);
         contactState.setRhoWeight(yoContactPoint, Double.NaN);
      }
   }
   
}
