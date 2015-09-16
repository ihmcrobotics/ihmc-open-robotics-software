package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.controlModules.LegStrengthCalculatorTools;
import us.ihmc.commonWalkingControlModules.controlModules.NewGeometricVirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.TeeterTotterLegStrengthCalculator;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;


public class GeometricStairsGroundReactionWrenchDistributor implements GroundReactionWrenchDistributor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final VirtualToePointCalculator virtualToePointCalculator;
   private final LegStrengthCalculator legStrengthCalculator;
   private final OldBipedSupportPolygons bipedSupportPolygons;
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();
   private final TranslationReferenceFrame centerOfPressurePlaneFrame;
   private final SideDependentList<Double> lambdas = new SideDependentList<Double>();
   private final SideDependentList<DoubleYoVariable> kValues = new SideDependentList<DoubleYoVariable>();
   private final double totalMass;
   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame centerOfMassFrame;

   private final SideDependentList<PlaneContactState> contactStates = new SideDependentList<PlaneContactState>();
   private final LinkedHashMap<PlaneContactState, FrameVector> forces = new LinkedHashMap<PlaneContactState, FrameVector>();
   private final LinkedHashMap<PlaneContactState, Double> normalTorques = new LinkedHashMap<PlaneContactState, Double>();
   private final SideDependentList<FramePoint2d> centersOfPressureInZUp = new SideDependentList<FramePoint2d>();
   private final SideDependentList<FramePoint2d> centersOfPressure = new SideDependentList<FramePoint2d>();

   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", registry);
   private final FramePoint2d centerOfPressure2d;
   private final FrameVector up;

   public GeometricStairsGroundReactionWrenchDistributor(CommonHumanoidReferenceFrames referenceFrames, OldBipedSupportPolygons bipedSupportPolygons,
           SideDependentList<? extends ContactablePlaneBody> bipedFeet, double totalMass, YoGraphicsListRegistry yoGraphicsListRegistry,
           YoVariableRegistry parentRegistry)
   {
      double maximumLegStrengthWhenTransferringAway = 0.95;
      this.virtualToePointCalculator = new NewGeometricVirtualToePointCalculator(referenceFrames, registry, yoGraphicsListRegistry,
              maximumLegStrengthWhenTransferringAway);
      this.legStrengthCalculator = new TeeterTotterLegStrengthCalculator(registry);
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.totalMass = totalMass;
      this.centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      this.centerOfPressurePlaneFrame = new TranslationReferenceFrame("centerOfPressure", centerOfMassFrame);
      this.centerOfPressure2d = new FramePoint2d(centerOfPressurePlaneFrame);
      this.feet = bipedFeet;

      for (RobotSide robotSide : RobotSide.values)
      {
         kValues.put(robotSide, new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "KValue", registry));
      }

      omega0.set(3.0);    // FIXME: hack to resolve circularity

      parentRegistry.addChild(registry);
      up = new FrameVector(centerOfMassFrame, 0.0, 0.0, 1.0);
   }

   public void solve(GroundReactionWrenchDistributorOutputData distributedWrench,
         GroundReactionWrenchDistributorInputData groundReactionWrenchDistributorInputData)
   {
      reset();
      
      List<PlaneContactState> contactStates = groundReactionWrenchDistributorInputData.getContactStates();
      
      for (PlaneContactState contactState : contactStates)
      {
         addContact(contactState);
      }
    
      SpatialForceVector desiredGroundReactionWrench = groundReactionWrenchDistributorInputData.getDesiredNetSpatialForceVector();
      RobotSide upcomingSupportleg = groundReactionWrenchDistributorInputData.getUpcomingSupportSide();
      this.solve(desiredGroundReactionWrench, upcomingSupportleg);
      
      this.getOutputData(distributedWrench);
   }
   
   private void reset()
   {
      // TODO: inefficient
      contactStates.clear();
      forces.clear();
      centersOfPressure.clear();
      centersOfPressureInZUp.clear();
      normalTorques.clear();
   }

   private void addContact(PlaneContactState contactState)
   {
      if (contactState == null) throw new RuntimeException("contactState == null");
      
      RobotSide robotSide = getRobotSide(contactState, feet);
      contactStates.put(robotSide, contactState);
      forces.put(contactState, new FrameVector(centerOfMassFrame));
      normalTorques.put(contactState, 0.0);
   }

   private void solve(SpatialForceVector desiredGroundReactionWrench, RobotSide upcomingSupportSide)
   {
      desiredGroundReactionWrench.changeFrame(centerOfMassFrame);

      // compute total force
      FrameVector force = desiredGroundReactionWrench.getLinearPartAsFrameVectorCopy();

      // compute fZ
      double fZ = force.getZ();

      // compute pseudoCoP, normalTorque
      FramePoint com = new FramePoint(centerOfMassFrame);
      double zCoP = com.getZ() - fZ / (totalMass * MathTools.square(omega0.getDoubleValue()));    // FIXME: hack to resolve circularity
      centerOfPressurePlaneFrame.updateTranslation(new FrameVector(centerOfMassFrame, 0.0, 0.0, zCoP));
      centerOfPressurePlaneFrame.update();
      double totalNormalTorque = centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure2d, desiredGroundReactionWrench,
                                    centerOfPressurePlaneFrame);
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      centerOfPressure2d.changeFrame(supportPolygonInMidFeetZUp.getReferenceFrame());
      GeometryTools.projectOntoPolygonAndCheckDistance(centerOfPressure2d, supportPolygonInMidFeetZUp, Double.POSITIVE_INFINITY);    // fix CoP slightly outside support polygon
      
      // compute deltaCMP
      FramePoint lineEnd = new FramePoint(com);
      lineEnd.add(force);
      FramePoint centerOfPressure = centerOfPressure2d.toFramePoint();
      centerOfPressure.changeFrame(com.getReferenceFrame());
      FramePoint cmp = GeometryTools.getIntersectionBetweenLineAndPlane(centerOfPressure, up, com, lineEnd);
      FrameVector deltaCMP = new FrameVector(cmp);
      deltaCMP.sub(centerOfPressure);

      // compute individual foot CoPs
      if (contactStates.size() == 2)
      {
         virtualToePointCalculator.packVirtualToePoints(centersOfPressureInZUp, bipedSupportPolygons, centerOfPressure2d, upcomingSupportSide);
      }
      
      else
      {
         RobotSide supportLeg = getSupportLeg(contactStates);
         FrameConvexPolygon2d footPolygonInAnkleZUp = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
         FramePoint2d footCenterOfPressure2d = new FramePoint2d(centerOfPressure2d);
         GeometryTools.projectOntoPolygonAndCheckDistance(footCenterOfPressure2d, footPolygonInAnkleZUp, 1e-10);    // fix numerical roundoff

         centersOfPressureInZUp.put(supportLeg, footCenterOfPressure2d);
         centersOfPressureInZUp.put(supportLeg.getOppositeSide(), new FramePoint2d(centerOfPressure2d.getReferenceFrame())); // arbitrary
      }

      SideDependentList<FramePoint> centersOfPressureOnSole3d = projectOntoSole(centersOfPressureInZUp);

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint footCoP = centersOfPressureOnSole3d.get(robotSide);
         if (footCoP != null)
         {
            footCoP.changeFrame(feet.get(robotSide).getSoleFrame());
            centersOfPressure.put(robotSide, footCoP.toFramePoint2d());
         }
         else
            centersOfPressure.put(robotSide, null);
      }

      // compute lambdas
      legStrengthCalculator.packLegStrengths(lambdas, centersOfPressureInZUp, centerOfPressure2d);
      
      // update omega0
      double omega0 = computeOmega0(com, fZ, centersOfPressureOnSole3d);
      this.omega0.set(omega0);

      // compute k values
      double k1PlusK2 = MathTools.square(omega0) * totalMass;
      for (RobotSide robotSide : RobotSide.values)
      {
         double k = lambdas.get(robotSide) * k1PlusK2;
         kValues.get(robotSide).set(k);
      }

      // compute moment weightings
      SideDependentList<Double> momentWeightings = new SideDependentList<Double>();
      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactState contactState = contactStates.get(robotSide);
         double momentWeighting;
         if (contactState == null || contactState.getContactFramePoints2dInContactCopy().size() == 0)
         {
            momentWeighting = 0.0;
         }
         else
         {
            List<FramePoint> contactPoints = contactState.getContactFramePointsInContactCopy();
            FramePoint centerOfPressureOnSole = centersOfPressureOnSole3d.get(robotSide);
            centerOfPressureOnSole.changeFrame(contactState.getPlaneFrame());
            double minDistance = GeometryTools.minimumDistance(centerOfPressureOnSole, contactPoints);
            momentWeighting = minDistance * lambdas.get(robotSide);            
         }
         momentWeightings.put(robotSide, momentWeighting);
      }

      LegStrengthCalculatorTools.normalize(momentWeightings);

      // compute individual foot forces and normal torques
      FramePoint groundReactionForceTerminalPoint = new FramePoint(centerOfMassFrame);
      groundReactionForceTerminalPoint.sub(deltaCMP);

      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactState contactState = contactStates.get(robotSide);

         FrameVector groundReactionForce = forces.get(contactState);
         if (groundReactionForce != null)
         {            
            groundReactionForce.setIncludingFrame(groundReactionForceTerminalPoint);
            FramePoint footCoP = centersOfPressureOnSole3d.get(robotSide);
            footCoP.changeFrame(groundReactionForce.getReferenceFrame());
            groundReactionForce.sub(footCoP);
            groundReactionForce.scale(kValues.get(robotSide).getDoubleValue());
         }

         // TODO: base on contact situation.
         double footNormalTorque = lambdas.get(robotSide) * totalNormalTorque;    // momentWeightings.get(robotSide) * totalNormalTorque;
         normalTorques.put(contactState, footNormalTorque);
      }
   }

   private void getOutputData(GroundReactionWrenchDistributorOutputData outputData)
   {
      outputData.reset();
      for (PlaneContactState planeContactState : contactStates)
      {
         if (planeContactState != null)
            outputData.set(planeContactState, getForce(planeContactState), getCenterOfPressure(planeContactState), getNormalTorque(planeContactState));
      }
   }
   
   private FrameVector getForce(PlaneContactState planeContactState)
   {
      return forces.get(planeContactState);
   }

   private FramePoint2d getCenterOfPressure(PlaneContactState contactState)
   {
      return centersOfPressure.get(getRobotSide(contactState, feet));
   }

   private double getNormalTorque(PlaneContactState contactState)
   {
      return normalTorques.get(contactState);
   }

   private double computeOmega0(FramePoint centerOfMass, double fZ, SideDependentList<FramePoint> virtualToePointsOnSole)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         virtualToePointsOnSole.get(robotSide).changeFrame(worldFrame);
      }

      FrameLine2d vtpToVTPLine = new FrameLine2d(virtualToePointsOnSole.get(RobotSide.LEFT).toFramePoint2d(),
                                    virtualToePointsOnSole.get(RobotSide.RIGHT).toFramePoint2d());

      FramePoint r1 = virtualToePointsOnSole.get(RobotSide.LEFT);
      FramePoint2d r12d = r1.toFramePoint2d();
      vtpToVTPLine.orthogonalProjection(r12d);    // not sure if necessary.
      double x1 = vtpToVTPLine.getParameterGivenPointEpsilon(r12d, 1e-12);
      double z1 = r1.getZ();

      FramePoint r2 = virtualToePointsOnSole.get(RobotSide.RIGHT);
      FramePoint2d r22d = r2.toFramePoint2d();
      vtpToVTPLine.orthogonalProjection(r22d);    // not sure if necessary.
      double x2 = vtpToVTPLine.getParameterGivenPointEpsilon(r22d, 1e-12);
      double z2 = r2.getZ();
      centerOfMass.changeFrame(worldFrame);
      double z = centerOfMass.getZ();

      double omega0Squared = (fZ * (x1 - x2))
                             / (totalMass
                                * (x1 * (z - lambdas.get(RobotSide.LEFT) * z1 + (-1 + lambdas.get(RobotSide.LEFT)) * z2)
                                   + x2 * (-z + z1 - lambdas.get(RobotSide.RIGHT) * z1 + lambdas.get(RobotSide.RIGHT) * z2)));

      if (omega0Squared <= 0.0)
         throw new RuntimeException("omega0Squared <= 0.0. omega0Squared = " + omega0Squared);

      double omega0 = Math.sqrt(omega0Squared);

      return omega0;
   }

   private static RobotSide getSupportLeg(SideDependentList<PlaneContactState> contactStates)
   {
      boolean inDoubleSupport = true;
      RobotSide supportSide = null;
      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactState contactState = contactStates.get(robotSide);
         if (contactState == null || !contactState.inContact())
         {
            inDoubleSupport = false;
         }
         else
         {
            supportSide = robotSide;
         }
      }

      if (supportSide == null)
         throw new RuntimeException("neither foot is a supporting foot");

      return inDoubleSupport ? null : supportSide;
   }

   private SideDependentList<FramePoint> projectOntoSole(SideDependentList<FramePoint2d> centersOfPressureInZUp)
   {
      SideDependentList<FramePoint> virtualToePointsOnSole = new SideDependentList<FramePoint>();
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint2d centerOfPressure = centersOfPressureInZUp.get(robotSide);
         if (centerOfPressure != null)
         {
            centerOfPressure.changeFrame(worldFrame);
            FramePoint virtualToePoint = centerOfPressure.toFramePoint();
            ReferenceFrame soleFrame = feet.get(robotSide).getSoleFrame();
            virtualToePoint = projectPointOntoSole(soleFrame, virtualToePoint);
            virtualToePointsOnSole.put(robotSide, virtualToePoint);
         }
      }

      return virtualToePointsOnSole;
   }

   private static FramePoint projectPointOntoSole(ReferenceFrame soleFrame, FramePoint footCoP)
   {
      FramePoint pointOnPlane = new FramePoint(soleFrame);
      FrameVector planeNormal = new FrameVector(soleFrame, 0.0, 0.0, 1.0);
      FramePoint lineStart = new FramePoint(footCoP);
      lineStart.changeFrame(soleFrame);
      FramePoint lineEnd = new FramePoint(footCoP);    // start at VTP
      lineEnd.setZ(lineEnd.getZ() - 1.0);    // down an arbitrary amount in the frame in which the VTP is expressed
      lineEnd.changeFrame(soleFrame);    // then change frame to sole frame

      return GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, lineStart, lineEnd);
   }

   private static RobotSide getRobotSide(PlaneContactState contactState, SideDependentList<? extends ContactablePlaneBody> feet)
   {
      RobotSide ret = null;
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);
         if (foot.getSoleFrame() == contactState.getPlaneFrame())
         {
            if (ret == null)
               ret = robotSide;
            else
               throw new RuntimeException();
         }
      }

      if (ret == null)
         throw new RuntimeException("robotSide for " + contactState + " could not be determined");

      return ret;
   }
}
