package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
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
 *
 */
public class GeometricFlatGroundReactionWrenchDistributor implements GroundReactionWrenchDistributorInterface
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final SideDependentList<PlaneContactState> contactStates = new SideDependentList<PlaneContactState>();
   private final SideDependentList<FrameConvexPolygon2d> convexPolygons = new SideDependentList<FrameConvexPolygon2d>();
   
   private final NewGeometricVirtualToePointCalculator virtualToePointCalculator;
   private final TeeterTotterLegStrengthCalculator legStrengthCalculator;
   
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();
   
   public GeometricFlatGroundReactionWrenchDistributor(YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
     ;
      this.virtualToePointCalculator = new NewGeometricVirtualToePointCalculator(parentRegistry, dynamicGraphicObjectsListRegistry, 0.95);
      this.legStrengthCalculator = new TeeterTotterLegStrengthCalculator(parentRegistry);
      
      virtualToePointCalculator.setAllFramesToComputeInToWorld();
   }
   
   public void reset()
   {
      // TODO Auto-generated method stub
      
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
      convexPolygons.clear();
      
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
         convexPolygons.set(robotSide, convexPolygon);
      }
      
      FramePoint centerOfPressureToPack = new FramePoint(worldFrame);
      
      
      // TODO: Finish This
//      centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressureToPack, desiredNetSpatialForceVector, centerOfPressurePlaneFrame)
//      
//      virtualToePointCalculator.packVirtualToePoints(virtualToePoints, copDesired, footPolygonsInMidFeetZUp, supportPolygonInMidFeetZUp, connectingEdge1, connectingEdge2, upcomingSupportSide)

      
   }

   public FramePoint2d getCenterOfPressure(PlaneContactState contactState)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public double getNormalTorque(PlaneContactState contactState)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   public FrameVector getForce(PlaneContactState contactState)
   {
      // TODO Auto-generated method stub
      return null;
   }

}
