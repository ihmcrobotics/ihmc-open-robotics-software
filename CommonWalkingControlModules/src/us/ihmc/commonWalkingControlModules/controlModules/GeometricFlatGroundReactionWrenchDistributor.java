package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2dAndConnectingEdges;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;


/**
 * Geometric based Wrench Distributor for Flat Ground. 
 * 
 * Contact States surface normals must be straight up, i.e. their reference frames must be ZUp Frames.
 * The Center of Pressure will be resolved in world frame, particularly with z=0 and ZUp surface normal.
 */
public class GeometricFlatGroundReactionWrenchDistributor implements GroundReactionWrenchDistributorInterface
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final SideDependentList<PlaneContactState> contactStates = new SideDependentList<PlaneContactState>();
   private final SideDependentList<FrameConvexPolygon2d> footConvexPolygons = new SideDependentList<FrameConvexPolygon2d>();
   
   private final NewGeometricVirtualToePointCalculator virtualToePointCalculator;
   private final TeeterTotterLegStrengthCalculator legStrengthCalculator;
   
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();
   
   private final SpatialForceVector desiredTotalForceVector = new SpatialForceVector();
   
   private final SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>(new FramePoint2d(worldFrame), new FramePoint2d(worldFrame));
   private final SideDependentList<Double> legStrengths = new SideDependentList<Double>();
   
   private final ReferenceFrame centerOfMassFrame;
   private final FrameVector gravitationalAcceleration;
   private final double mass;
   
   public GeometricFlatGroundReactionWrenchDistributor(ReferenceFrame centerOfMassFrame, FrameVector gravitationalAcceleration, double mass, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.virtualToePointCalculator = new NewGeometricVirtualToePointCalculator(parentRegistry, dynamicGraphicObjectsListRegistry, 0.95);
      this.legStrengthCalculator = new TeeterTotterLegStrengthCalculator(parentRegistry);
      
      virtualToePointCalculator.setAllFramesToComputeInToWorld();
      
      this.centerOfMassFrame = centerOfMassFrame;
      this.gravitationalAcceleration = gravitationalAcceleration;
      this.mass = mass;
   }
   
   public void reset()
   {
      contactStates.clear();
      footConvexPolygons.clear();
   }

   public void addContact(PlaneContactState contactState, double coefficientOfFriction, double rotationalCoefficientOfFriction)
   {
      if (!contactState.getPlaneFrame().isZupFrame()) throw new RuntimeException("GeometricFlatGroundReactionWrenchDistributor: Must be a ZUpFrame!");
      
      if (contactStates.get(RobotSide.LEFT) == null)
      {
         contactStates.set(RobotSide.LEFT, contactState);
      }
      else if (contactStates.get(RobotSide.RIGHT) == null)
      {
         contactStates.set(RobotSide.RIGHT, contactState);
      }
         
      else
      {
         throw new RuntimeException("GeometricFlatGroundReactionWrenchDistributor only works with 2 flat feet. First one added is left, second one right");
      }
      
   }

   public void solve(SpatialForceVector desiredNetSpatialForceVector)
   {
      this.desiredTotalForceVector.set(desiredNetSpatialForceVector);
      footConvexPolygons.clear();
      
      for (RobotSide robotSide : RobotSide.values())
      {
         PlaneContactState contactState = contactStates.get(robotSide);
         
         List<FramePoint> contactPoints = contactState.getContactPoints();
         ArrayList<FramePoint2d> projectionsOnGround = new ArrayList<FramePoint2d>();
         
         for (FramePoint framePoint : contactPoints)
         {
            FramePoint pointInWorld = framePoint.changeFrameCopy(worldFrame);
            projectionsOnGround.add(pointInWorld.toFramePoint2d());
         }
         
         FrameConvexPolygon2d convexPolygon = new FrameConvexPolygon2d(projectionsOnGround);
         footConvexPolygons.set(robotSide, convexPolygon);
      }
      
      FramePoint centerOfPressure = new FramePoint(worldFrame);

      double normalTorque = centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure, desiredTotalForceVector, worldFrame);

      FrameConvexPolygon2d leftFootPolygon = footConvexPolygons.get(RobotSide.LEFT);
      FrameConvexPolygon2d rightFootPolygon = footConvexPolygons.get(RobotSide.RIGHT);
      
      FrameConvexPolygon2dAndConnectingEdges supportPolygonAndConnectingEdges = FrameConvexPolygon2d.combineDisjointPolygons(leftFootPolygon, rightFootPolygon);
      
      FrameConvexPolygon2d supportPolygon = supportPolygonAndConnectingEdges.getFrameConvexPolygon2d();
      FrameLineSegment2d connectingEdge1 = supportPolygonAndConnectingEdges.getConnectingEdge1();
      FrameLineSegment2d connectingEdge2 = supportPolygonAndConnectingEdges.getConnectingEdge2();
      
      RobotSide upcomingSupportSide = RobotSide.LEFT; //null;
      FramePoint2d centerOfPressure2d = centerOfPressure.toFramePoint2d();
      
      virtualToePointCalculator.packVirtualToePoints(virtualToePoints, centerOfPressure2d, footConvexPolygons, supportPolygon, connectingEdge1, connectingEdge2, upcomingSupportSide);
      legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, centerOfPressure2d);   
   }

   public FramePoint2d getCenterOfPressure(PlaneContactState contactState)
   {
      if (contactStates.get(RobotSide.LEFT) == contactState)
      {
         return new FramePoint2d(virtualToePoints.get(RobotSide.LEFT));
      }
      else if (contactStates.get(RobotSide.LEFT) == contactState)
      {
         return new FramePoint2d(virtualToePoints.get(RobotSide.RIGHT));
      }
      else throw new RuntimeException("Don't have that contact state in my contact states!");
   }

   public double getNormalTorque(PlaneContactState contactState)
   {
      RobotSide robotSide = getRobotSide(contactState);
      
      SpatialForceVector temporaryForceVector = new SpatialForceVector(desiredTotalForceVector);
      
      temporaryForceVector.changeFrame(contactState.getPlaneFrame());
      FrameVector torque = new FrameVector(contactState.getPlaneFrame(), temporaryForceVector.getAngularPartCopy());
      torque.scale(legStrengths.get(robotSide));

      return torque.getZ();
   }

   public FrameVector getForce(PlaneContactState contactState)
   {
      RobotSide robotSide = getRobotSide(contactState);
      
      SpatialForceVector temporaryForceVector = new SpatialForceVector(desiredTotalForceVector);
      
      temporaryForceVector.changeFrame(contactState.getPlaneFrame());
      FrameVector force = new FrameVector(contactState.getPlaneFrame(), temporaryForceVector.getLinearPartCopy());
      force.scale(legStrengths.get(robotSide));
     
      return force;
   }
   
   private RobotSide getRobotSide(PlaneContactState contactState)
   {
      if (contactStates.get(RobotSide.LEFT) == contactState)
      {
         return RobotSide.LEFT;
      }
      else if (contactStates.get(RobotSide.LEFT) == contactState)
      {
         return RobotSide.RIGHT;
      }
      else throw new RuntimeException("Don't have that contact state in my contact states!");
      
   }

}
