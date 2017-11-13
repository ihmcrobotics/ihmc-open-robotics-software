package us.ihmc.commonWalkingControlModules.controlModules.foot.contactPoints;

import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameLineSegment;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class will project contact points on to a contact line and interpolate them back to the correct place over a specified duration.
 * This would be similar to a rolling foot, where the contact polygon grows as the foot comes in contact.
 */
public class ContactStateInterpolator
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
   private final double dt;
   

   /**
    * This class will project contact points on to a contact line and interpolate them back to the correct place over a specified duration.
    * @param robotSide - used for naming only
    * @param contactState - the contact state to interpolate
    * @param dt - the dt used to increment the time in trajectory
    * @param parentRegistry
    */
   public ContactStateInterpolator(RobotSide robotSide, YoPlaneContactState contactState, double dt, YoVariableRegistry parentRegistry)
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
      
      for(YoContactPoint yoContactPoint : contactPoints)
      {
         FramePoint2D originalPosition = new FramePoint2D();
         yoContactPoint.getPosition2d(originalPosition);
         
         initialPositions.put(yoContactPoint, new FramePoint2D());
         finalPositions.put(yoContactPoint, originalPosition);
      }
      
      parentRegistry.addChild(registry);
   }
   
   /**
    * projects the contact points NOT in contact onto the contact line and sets them in contact
    * @param duration - the time it takes to interpolate the contact points from the line to their original position
    * @param contactLine the line formed by two contact points in contact
    */
   public void initialize(double duration, FrameLineSegment contactLine)
   {
      contactLine.changeFrame(planeFrame);
      contactLine.getIncludingFrame(contactLine2D);
      
      this.duration.set(duration);
      timeInTrajectory.set(0.0);
      
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
            yoContactPoint.setInContact(true);
         }
      }
      contactState.updateInContact();
   }

   /**
    * moves the contact points from their projected positions to their original positions
    */
   public void update()
   {
      timeInTrajectory.add(dt);
      alpha.set(timeInTrajectory.getDoubleValue() / duration.getDoubleValue());
      alpha.set(MathTools.clamp(alpha.getDoubleValue(), 0.0, 1.0));
      
      for(int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint yoContactPoint = contactPoints.get(i);
         if(contactPointInterpolationActivated[i])
         {
               FramePoint2D initialPosition = initialPositions.get(yoContactPoint);
               FramePoint2D finalPosition = finalPositions.get(yoContactPoint);
               
               contactPointPosition.changeFrame(planeFrame);
               contactPointPosition.interpolate(initialPosition, finalPosition, alpha.getDoubleValue());
               contactPointPosition.changeFrame(yoContactPoint.getReferenceFrame());
               yoContactPoint.setPosition2d(contactPointPosition);
         }
      }
      
      //TODO: once the new icp planner handles replans, enable notifying the planner for a replan and test it out
      //      contactState.notifyContactStateHasChanged();
   }
   
   public boolean isDone()
   {
      return timeInTrajectory.getDoubleValue() >= duration.getDoubleValue();
   }

   /**
    * Moves the contact points back to there original positions
    */
   public void resetContactState()
   {
      for(int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint yoContactPoint = contactPoints.get(i);
         FramePoint2D finalPosition = finalPositions.get(yoContactPoint);
         contactPointPosition.setIncludingFrame(finalPosition);
         contactPointPosition.changeFrame(yoContactPoint.getReferenceFrame());
         yoContactPoint.setPosition2d(contactPointPosition);
      }
   }
}
