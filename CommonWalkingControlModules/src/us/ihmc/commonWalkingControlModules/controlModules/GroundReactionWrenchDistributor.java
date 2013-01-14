package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactState;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class GroundReactionWrenchDistributor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final CommonWalkingReferenceFrames referenceFrames;
   private final VirtualToePointCalculator virtualToePointCalculator;
   private final LegStrengthCalculator legStrengthCalculator;
   private final SideDependentList<Double> lambdas = new SideDependentList<Double>();
   private final SideDependentList<DoubleYoVariable> kValues = new SideDependentList<DoubleYoVariable>();
   private final double totalMass;
   private final SideDependentList<RigidBody> feet = new SideDependentList<RigidBody>();
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private HashMap<RigidBody, Wrench> groundReactionWrenches;    // TODO: garbage
   private SideDependentList<FramePoint> virtualToePointsOnSole;    // TODO: garbage
   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", registry);

   public GroundReactionWrenchDistributor(CommonWalkingReferenceFrames referenceFrames, FullRobotModel fullRobotModel,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.virtualToePointCalculator = new NewGeometricVirtualToePointCalculator(referenceFrames, registry, dynamicGraphicObjectsListRegistry, 0.95);
      this.legStrengthCalculator = new TeeterTotterLegStrengthCalculator(registry);
      this.referenceFrames = referenceFrames;
      this.totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      for (RobotSide robotSide : RobotSide.values())
      {
         this.feet.put(robotSide, fullRobotModel.getFoot(robotSide));
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         kValues.put(robotSide, new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "KValue", registry));
      }

      parentRegistry.addChild(registry);
   }

   public void distributeGroundReactionWrench(FramePoint2d desiredCoP, FrameVector2d desiredDeltaCMP, double fZ, FrameVector totalgroundReactionMoment,
           SideDependentList<ContactState> contactStates, BipedSupportPolygons bipedSupportPolygons, RobotSide upcomingSupportLeg)
   {
      SideDependentList<FramePoint2d> virtualToePoints = computeVirtualToePoints(desiredCoP, contactStates, bipedSupportPolygons, upcomingSupportLeg);
      legStrengthCalculator.packLegStrengths(lambdas, virtualToePoints, desiredCoP);
      virtualToePointsOnSole = projectOntoSole(virtualToePoints);

      FramePoint centerOfMass = new FramePoint(referenceFrames.getCenterOfMassFrame());
      double omega0 = computeOmega0(centerOfMass, fZ, virtualToePointsOnSole);
      this.omega0.set(omega0);

      double k1PlusK2 = MathTools.square(omega0) * totalMass;
      for (RobotSide robotSide : RobotSide.values())
      {
         double k = lambdas.get(robotSide) * k1PlusK2;
         kValues.get(robotSide).set(k);
      }

      groundReactionWrenches = computeGroundReactionWrenches(desiredDeltaCMP, virtualToePointsOnSole, totalgroundReactionMoment, contactStates);
   }

   public HashMap<RigidBody, Wrench> getGroundReactionWrenches()
   {
      return groundReactionWrenches;
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }
   
   public SideDependentList<FramePoint> getVirtualToePointsOnSole()
   {
      return virtualToePointsOnSole;
   }

   private HashMap<RigidBody, Wrench> computeGroundReactionWrenches(FrameVector2d desiredDeltaCMP, SideDependentList<FramePoint> virtualToePointsOnSole,
           FrameVector totalgroundReactionMoment, SideDependentList<ContactState> contactStates)
   {
      SideDependentList<Double> momentWeightings = new SideDependentList<Double>();
      for (RobotSide robotSide : RobotSide.values())
      {
         ContactState contactState = contactStates.get(robotSide);
         FramePoint virtualToePoint = virtualToePointsOnSole.get(robotSide);
         virtualToePoint.changeFrame(contactState.getBodyFrame());
         List<FramePoint> contactPoints = contactState.getContactPoints();
         double momentWeighting;
         if (contactPoints.size() > 0)
         {
            double minDistance = GeometryTools.minimumDistance(virtualToePoint, contactPoints);
            momentWeighting = minDistance * lambdas.get(robotSide);
         }
         else
            momentWeighting = 0.0;

         momentWeightings.put(robotSide, momentWeighting);
      }

      LegStrengthCalculatorTools.normalize(momentWeightings);

      HashMap<RigidBody, Wrench> groundReactionWrenches = new HashMap<RigidBody, Wrench>();
      for (RobotSide robotSide : RobotSide.values())
      {
         FramePoint groundReactionForceTerminalPoint = new FramePoint(referenceFrames.getCenterOfMassFrame());
         groundReactionForceTerminalPoint.changeFrame(worldFrame);
         desiredDeltaCMP.changeFrame(worldFrame);
         groundReactionForceTerminalPoint.setX(groundReactionForceTerminalPoint.getX() - desiredDeltaCMP.getX());
         groundReactionForceTerminalPoint.setY(groundReactionForceTerminalPoint.getY() - desiredDeltaCMP.getY());
         FrameVector groundReactionForce = new FrameVector(groundReactionForceTerminalPoint);

         FramePoint virtualToePoint = virtualToePointsOnSole.get(robotSide);
         virtualToePoint.changeFrame(worldFrame);
         groundReactionForce.sub(virtualToePoint);
         groundReactionForce.scale(kValues.get(robotSide).getDoubleValue());

         FrameVector groundReactionMoment = new FrameVector(totalgroundReactionMoment);

         // TODO: base on contact situation.
         groundReactionMoment.scale(lambdas.get(robotSide));    // momentWeightings.get(robotSide));


         RigidBody foot = feet.get(robotSide);
         ReferenceFrame footCoMFrame = foot.getBodyFixedFrame();
         FrameVector torque = new FrameVector(worldFrame);
         torque.cross(virtualToePoint, groundReactionForce);
         Wrench groundReactionWrench = new Wrench(footCoMFrame, worldFrame, groundReactionForce.getVector(), torque.getVector());
         groundReactionWrench.addAngularPart(groundReactionMoment.getVector());
         groundReactionWrenches.put(foot, groundReactionWrench);
      }

      return groundReactionWrenches;
   }

   private double computeOmega0(FramePoint centerOfMass, double fZ, SideDependentList<FramePoint> virtualToePointsOnSole)
   {
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

   private SideDependentList<FramePoint2d> computeVirtualToePoints(FramePoint2d desiredCoP, SideDependentList<ContactState> contactStates,
           BipedSupportPolygons bipedSupportPolygons, RobotSide upcomingSupportLeg)
   {
      SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>();

      RobotSide supportLeg = getSupportLeg(contactStates);
      if (supportLeg == null)
      {
         virtualToePointCalculator.packVirtualToePoints(virtualToePoints, bipedSupportPolygons, desiredCoP, upcomingSupportLeg);
      }
      else
      {
         virtualToePointCalculator.hideVisualizationGraphics();
         FrameConvexPolygon2d footPolygonInAnkleZUp = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
         GeometryTools.projectOntoPolygonAndCheckDistance(desiredCoP, footPolygonInAnkleZUp, 1e-10);    // fix numerical roundoff

         virtualToePoints.put(supportLeg, desiredCoP);
         virtualToePoints.put(supportLeg.getOppositeSide(), new FramePoint2d(desiredCoP.getReferenceFrame()));
      }

      return virtualToePoints;
   }

   private static RobotSide getSupportLeg(SideDependentList<ContactState> contactStates)
   {
      boolean inDoubleSupport = true;
      RobotSide supportSide = null;
      for (RobotSide robotSide : RobotSide.values())
      {
         ContactState contactState = contactStates.get(robotSide);
         if (!contactState.inContact())
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

   private SideDependentList<FramePoint> projectOntoSole(SideDependentList<FramePoint2d> virtualToePoints)
   {
      SideDependentList<FramePoint> virtualToePointsOnSole = new SideDependentList<FramePoint>();
      for (RobotSide robotSide : RobotSide.values())
      {
         virtualToePoints.get(robotSide).changeFrame(worldFrame);
         FramePoint virtualToePoint = virtualToePoints.get(robotSide).toFramePoint();
         virtualToePoint = projectPointOntoSole(robotSide, virtualToePoint);
         virtualToePoint.changeFrame(worldFrame);
         virtualToePointsOnSole.put(robotSide, virtualToePoint);
      }

      return virtualToePointsOnSole;
   }

   private FramePoint projectPointOntoSole(RobotSide robotSide, FramePoint virtualToePoint)
   {
      ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
      FramePoint pointOnPlane = new FramePoint(soleFrame);
      FrameVector planeNormal = new FrameVector(soleFrame, 0.0, 0.0, 1.0);
      FramePoint lineStart = virtualToePoint.changeFrameCopy(soleFrame);
      FramePoint lineEnd = new FramePoint(virtualToePoint);    // start at VTP
      lineEnd.setZ(lineEnd.getZ() - 1.0);    // down an arbitrary amount in the frame in which the VTP is expressed
      lineEnd.changeFrame(soleFrame);    // then change frame to sole frame

      return GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, lineStart, lineEnd);
   }
}
