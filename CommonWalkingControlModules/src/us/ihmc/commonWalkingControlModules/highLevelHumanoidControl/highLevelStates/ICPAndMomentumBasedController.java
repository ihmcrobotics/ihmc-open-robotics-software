package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.ConstantOmega0Calculator;
import us.ihmc.commonWalkingControlModules.calculators.Omega0Calculator;
import us.ihmc.commonWalkingControlModules.calculators.Omega0CalculatorInterface;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public class ICPAndMomentumBasedController
{
   private static final boolean USE_CONSTANT_OMEGA0 = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final MomentumBasedController momentumBasedController;

   private final SideDependentList<? extends ContactablePlaneBody> bipedFeet;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoFramePoint2d desiredICP;
   private final YoFrameVector2d desiredICPVelocity;
   private final EnumYoVariable<RobotSide> supportLeg;
   private final DoubleYoVariable controlledCoMHeightAcceleration;
   private final YoFramePoint yoCapturePoint;
   private final DoubleYoVariable omega0;
   private final Omega0CalculatorInterface omega0Calculator;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   
   public ICPAndMomentumBasedController(MomentumBasedController momentumBasedController, FullRobotModel fullRobotModel,
           SideDependentList<? extends ContactablePlaneBody> bipedFeet, BipedSupportPolygons bipedSupportPolygons, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      parentRegistry.addChild(registry);

      this.momentumBasedController = momentumBasedController;

      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      if (USE_CONSTANT_OMEGA0)
      {
         double constantOmega0 = 3.4;
         this.omega0Calculator = new ConstantOmega0Calculator(constantOmega0, registry);
      }
      else
      {
         ReferenceFrame centerOfMassFrame = momentumBasedController.getCenterOfMassFrame();
         this.omega0Calculator = new Omega0Calculator(centerOfMassFrame, totalMass);
      }

      omega0 = new DoubleYoVariable("omega0", registry);
      yoCapturePoint = new YoFramePoint("capturePoint", worldFrame, registry);
      this.controlledCoMHeightAcceleration = new DoubleYoVariable("controlledCoMHeightAcceleration", registry);
      this.bipedFeet = bipedFeet;
      this.bipedSupportPolygons = bipedSupportPolygons;

      desiredICP = new YoFramePoint2d("desiredICP", "", worldFrame, registry);
      desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", worldFrame, registry);

      supportLeg = EnumYoVariable.create("supportLeg", "", RobotSide.class, registry, true);

      // TODO: Have local updatables, instead of adding them to the momentum based controller.
      
      this.updatables.add(new Omega0Updater());
      this.updatables.add(new BipedSupportPolygonsUpdater());
      this.updatables.add(new CapturePointUpdater());

      if (dynamicGraphicObjectsListRegistry != null)
      {
         ArrayList<PlaneContactState> feetContactStates = new ArrayList<PlaneContactState>();
         momentumBasedController.getFeetContactStates(feetContactStates);
//         Collection<PlaneContactState> planeContactStates = momentumBasedController.getPlaneContactStates();
         FootPolygonVisualizer footPolygonVisualizer = new FootPolygonVisualizer(feetContactStates, dynamicGraphicObjectsListRegistry, registry);
         momentumBasedController.addUpdatable(footPolygonVisualizer);
         
         DynamicGraphicPosition capturePointViz = yoCapturePoint.createDynamicGraphicPosition("Capture Point", 0.01, YoAppearance.Blue(),
                                                     GraphicType.ROTATED_CROSS);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("Capture Point", capturePointViz);
         dynamicGraphicObjectsListRegistry.registerArtifact("Capture Point", capturePointViz.createArtifact());
      }
      
      capturePointOffsetY.set(0.0);
   }
   
   public void updateUpdatables(double time)
   {
      for (int i=0; i<updatables.size(); i++)
      {
         updatables.get(i).update(time);
      }
   }

   public void initialize()
   {
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   private final FramePoint centerOfMassPosition = new FramePoint(worldFrame);
   private final FrameVector centerOfMassVelocity = new FrameVector(worldFrame);

   private final FramePoint2d capturePoint2d = new FramePoint2d(worldFrame);
   private final FramePoint2d centerOfMassPosition2d = new FramePoint2d(worldFrame);
   private final FrameVector2d centerOfMassVelocity2d = new FrameVector2d(worldFrame);
   
   //
   private final DoubleYoVariable capturePointOffsetY = new DoubleYoVariable("capturePointOffsetY", registry);
   private final FrameVector2d capturePointOffsetHack2d = new FrameVector2d();
   
   protected void computeCapturePoint()
   {
      centerOfMassPosition.setToZero(momentumBasedController.getCenterOfMassFrame());
      momentumBasedController.getCenterOfMassJacobian().packCenterOfMassVelocity(centerOfMassVelocity);
      
      centerOfMassPosition.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      centerOfMassPosition2d.set(centerOfMassPosition.getX(), centerOfMassPosition.getY());
      centerOfMassVelocity2d.set(centerOfMassVelocity.getX(), centerOfMassVelocity.getY());
      
      CapturePointCalculator.computeCapturePoint(capturePoint2d, centerOfMassPosition2d, centerOfMassVelocity2d, getOmega0());
      
      ReferenceFrame midFeetZUpFrame = momentumBasedController.getReferenceFrames().getMidFeetZUpFrame();
      capturePointOffsetHack2d.setIncludingFrame(midFeetZUpFrame, 0.0, capturePointOffsetY.getDoubleValue());
      capturePointOffsetHack2d.changeFrame(capturePoint2d.getReferenceFrame());
      capturePoint2d.add(capturePointOffsetHack2d);
      
      capturePoint2d.changeFrame(yoCapturePoint.getReferenceFrame());
      yoCapturePoint.set(capturePoint2d.getX(), capturePoint2d.getY(), 0.0);
   }


   private final SideDependentList<List<FramePoint>> footContactPoints = new SideDependentList<List<FramePoint>>(new ArrayList<FramePoint>(), new ArrayList<FramePoint>());
//   private final SideDependentList<PlaneContactState> footContactStates = new SideDependentList<PlaneContactState>();
   
   protected void updateBipedSupportPolygons(BipedSupportPolygons bipedSupportPolygons)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         momentumBasedController.getContactPoints(bipedFeet.get(robotSide), footContactPoints.get(robotSide));
//         footContactStates.put(robotSide, momentumBasedController.getContactState(bipedFeet.get(robotSide)));
      }

      bipedSupportPolygons.update(footContactPoints, true);
//      bipedSupportPolygons.update(footContactStates);
   }

   private final class Omega0Updater implements Updatable
   {
      public void update(double time)
      {
         List<FramePoint2d> cops = new ArrayList<FramePoint2d>();
         for (ContactablePlaneBody foot : bipedFeet.values())
         {
            FramePoint2d coP = momentumBasedController.getCoP(foot);
            if (coP != null)
               cops.add(coP);
         }

         ReferenceFrame centerOfMassFrame = momentumBasedController.getCenterOfMassFrame();
         SpatialForceVector admissibleGroundReactionWrench = new SpatialForceVector(centerOfMassFrame);

//       FrameVector force = admissibleDesiredGroundReactionForce.getFrameVectorCopy();
         FrameVector force = momentumBasedController.getAdmissibleDesiredGroundReactionForceCopy();

         force.changeFrame(admissibleGroundReactionWrench.getExpressedInFrame());
         admissibleGroundReactionWrench.setLinearPart(force.getVector());

//       FrameVector torque = admissibleDesiredGroundReactionTorque.getFrameVectorCopy();
         FrameVector torque = momentumBasedController.getAdmissibleDesiredGroundReactionTorqueCopy();

         torque.changeFrame(admissibleGroundReactionWrench.getExpressedInFrame());
         admissibleGroundReactionWrench.setAngularPart(torque.getVector());

         SpatialForceVector gravitationalWrench = momentumBasedController.getGravitationalWrench();
         if (admissibleGroundReactionWrench.getLinearPartCopy().getZ() == 0.0)
            admissibleGroundReactionWrench.set(gravitationalWrench);    // FIXME: hack to resolve circularity

         omega0.set(omega0Calculator.computeOmega0(cops, admissibleGroundReactionWrench));
      }
   }


   private final class BipedSupportPolygonsUpdater implements Updatable
   {
      public void update(double time)
      {
         updateBipedSupportPolygons(bipedSupportPolygons);
      }
   }


   private class CapturePointUpdater implements Updatable
   {
      public void update(double time)
      {
         computeCapturePoint();
      }
   }


   // TODO: Following has been added for big refactor. Need to be checked.

   public EnumYoVariable<RobotSide> getYoSupportLeg()
   {
      return supportLeg;
   }

   public DoubleYoVariable getControlledCoMHeightAcceleration()
   {
      return controlledCoMHeightAcceleration;
   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public YoFramePoint getCapturePoint()
   {
      return yoCapturePoint;
   }

   public YoFramePoint2d getDesiredICP()
   {
      return desiredICP;
   }
   
   public void getDesiredICP(FramePoint2d pointToPack)
   {
      this.desiredICP.getFrameTuple2d(pointToPack);
   }

   public YoFrameVector2d getDesiredICPVelocity()
   {
      return desiredICPVelocity;
   }

   public SideDependentList<? extends ContactablePlaneBody> getBipedFeet()
   {
      return bipedFeet;
   }

   public ArrayList<Updatable> getUpdatables()
   {
      return updatables;
   }
}
