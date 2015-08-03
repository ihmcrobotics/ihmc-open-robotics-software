package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.controlModules.NewGeometricVirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.TeeterTotterLegStrengthCalculator;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2dAndConnectingEdges;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.PoseReferenceFrame;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;



/**
 * Geometric based Wrench Distributor for Flat Ground. 
 * 
 * Contact States surface normals must be straight up, i.e. their reference frames must be ZUp Frames.
 * The Center of Pressure will be resolved in world frame, particularly with z=0 and ZUp surface normal.
 */
public class GeometricFlatGroundReactionWrenchDistributor implements GroundReactionWrenchDistributor
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
   
   public GeometricFlatGroundReactionWrenchDistributor(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.virtualToePointCalculator = new NewGeometricVirtualToePointCalculator(parentRegistry, yoGraphicsListRegistry, 0.95);
      this.legStrengthCalculator = new TeeterTotterLegStrengthCalculator(parentRegistry);
      
      virtualToePointCalculator.setAllFramesToComputeInToWorld();
   }
   
   public void solve(GroundReactionWrenchDistributorOutputData distributedWrench,
         GroundReactionWrenchDistributorInputData groundReactionWrenchDistributorInputData)
   {
      reset();
      
      List<PlaneContactState> contactStates = groundReactionWrenchDistributorInputData.getContactStates();
   
      for (PlaneContactState contactState : contactStates)
      {
         addContact(contactState, contactState.getCoefficientOfFriction());
      }
    
      SpatialForceVector desiredGroundReactionWrench = groundReactionWrenchDistributorInputData.getDesiredNetSpatialForceVector();
      RobotSide upcomingSupportleg = groundReactionWrenchDistributorInputData.getUpcomingSupportSide();
      this.solve(desiredGroundReactionWrench, upcomingSupportleg);
      
      this.getOutputData(distributedWrench);
   }
   
   private void reset()
   {
      contactStates.clear();      
      footConvexPolygons.clear();
   }

   private void addContact(PlaneContactState contactState, double coefficientOfFriction)
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

   private void solve(SpatialForceVector desiredGroundReactionWrench, RobotSide upcomingSupportLeg)
   {
      this.desiredTotalForceVector.set(desiredGroundReactionWrench);
      footConvexPolygons.clear();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactState contactState = contactStates.get(robotSide);
         
         List<FramePoint> contactPoints = contactState.getContactFramePointsInContactCopy();
         ArrayList<FramePoint2d> projectionsOnGround = new ArrayList<FramePoint2d>();
         
         for (FramePoint framePoint : contactPoints)
         {
            FramePoint pointInWorld = new FramePoint(framePoint);
            pointInWorld.changeFrame(worldFrame);
            projectionsOnGround.add(pointInWorld.toFramePoint2d());
         }
         
         FrameConvexPolygon2d convexPolygon = new FrameConvexPolygon2d(projectionsOnGround);
         footConvexPolygons.set(robotSide, convexPolygon);
      }
      
      FramePoint2d centerOfPressure2d = new FramePoint2d(worldFrame);

      @SuppressWarnings("unused") // We don't use the normal torque for anything here. But we do use the other things.
      double normalTorque = centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure2d, desiredTotalForceVector, worldFrame);

      FrameConvexPolygon2d leftFootPolygon = footConvexPolygons.get(RobotSide.LEFT);
      FrameConvexPolygon2d rightFootPolygon = footConvexPolygons.get(RobotSide.RIGHT);
      
      FrameConvexPolygon2dAndConnectingEdges supportPolygonAndConnectingEdges = ConvexPolygonTools.combineDisjointPolygons(leftFootPolygon, rightFootPolygon);
      
      FrameConvexPolygon2d supportPolygon = supportPolygonAndConnectingEdges.getFrameConvexPolygon2d();
      FrameLineSegment2d connectingEdge1 = supportPolygonAndConnectingEdges.getConnectingEdge1();
      FrameLineSegment2d connectingEdge2 = supportPolygonAndConnectingEdges.getConnectingEdge2();
            
      boolean needToProject = !supportPolygon.isPointInside(centerOfPressure2d);
      if (needToProject)
      {
         supportPolygon.orthogonalProjection(centerOfPressure2d);
      }
      virtualToePointCalculator.packVirtualToePoints(virtualToePoints, centerOfPressure2d, footConvexPolygons, supportPolygon, connectingEdge1, connectingEdge2, upcomingSupportLeg);
      legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, centerOfPressure2d);
      
      // Verify that virtual toe points and leg strength percentages give the overall force:
   }

   private void getOutputData(GroundReactionWrenchDistributorOutputData outputData)
   {
      outputData.reset();
      for (PlaneContactState planeContactState : contactStates)
      {
         outputData.set(planeContactState, getForceCopy(planeContactState), getCenterOfPressureCopy(planeContactState), getNormalTorque(planeContactState));
      }
   }
   
   private FramePoint2d getCenterOfPressureCopy(PlaneContactState contactState)
   {
      RobotSide robotSide = getRobotSide(contactState);
      return new FramePoint2d(virtualToePoints.get(robotSide));
   }

   private double getNormalTorque(PlaneContactState contactState)
   {
      RobotSide robotSide = getRobotSide(contactState);
      
      SpatialForceVector temporaryForceVector = new SpatialForceVector(desiredTotalForceVector);
      temporaryForceVector.scale(legStrengths.get(robotSide));
      
      FramePoint2d virtualToePoint = virtualToePoints.get(robotSide);
      ReferenceFrame virtualToePointFrame = createVTPReferenceFrame(virtualToePoint);
      
      temporaryForceVector.changeFrame(virtualToePointFrame);
      FrameVector torque = new FrameVector(virtualToePointFrame, temporaryForceVector.getAngularPartCopy());

      return torque.getZ();
   }

   private FrameVector getForceCopy(PlaneContactState contactState)
   {
      RobotSide robotSide = getRobotSide(contactState);
      
      SpatialForceVector temporaryForceVector = new SpatialForceVector(desiredTotalForceVector);
      
      temporaryForceVector.changeFrame(contactState.getPlaneFrame());
      FrameVector force = new FrameVector(contactState.getPlaneFrame(), temporaryForceVector.getLinearPartCopy());
      force.scale(legStrengths.get(robotSide));
     
      return force;
   }
   
   private ReferenceFrame createVTPReferenceFrame(FramePoint2d virtualToePoint2d)
   {
      PoseReferenceFrame vtpFrame = new PoseReferenceFrame("vtpFrame", virtualToePoint2d.getReferenceFrame());
      
      FramePoint position = new FramePoint(virtualToePoint2d.getReferenceFrame());
      position.setXY(virtualToePoint2d);
      FrameOrientation orientation = new FrameOrientation(virtualToePoint2d.getReferenceFrame());
      vtpFrame.setPoseAndUpdate(position, orientation);
      vtpFrame.update();
      
      return vtpFrame;
   }
   
   private RobotSide getRobotSide(PlaneContactState contactState)
   {
      if (contactStates.get(RobotSide.LEFT) == contactState)
      {
         return RobotSide.LEFT;
      }
      else if (contactStates.get(RobotSide.RIGHT) == contactState)
      {
         return RobotSide.RIGHT;
      }
      else throw new RuntimeException("Don't have that contact state in my contact states!");
   }

}
