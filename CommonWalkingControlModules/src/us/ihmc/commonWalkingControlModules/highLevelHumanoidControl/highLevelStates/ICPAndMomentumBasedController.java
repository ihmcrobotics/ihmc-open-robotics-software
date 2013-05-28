package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.ConstantOmega0Calculator;
import us.ihmc.commonWalkingControlModules.calculators.Omega0Calculator;
import us.ihmc.commonWalkingControlModules.calculators.Omega0CalculatorInterface;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
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
   private final DoubleYoVariable desiredCoMHeightAcceleration;
   private final YoFramePoint capturePoint;
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
      capturePoint = new YoFramePoint("capturePoint", worldFrame, registry);
      this.desiredCoMHeightAcceleration = new DoubleYoVariable("desiredCoMHeightAcceleration", registry);
      this.bipedFeet = bipedFeet;
      this.bipedSupportPolygons = bipedSupportPolygons;

      desiredICP = new YoFramePoint2d("desiredICP", "", ReferenceFrame.getWorldFrame(), registry);
      desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", ReferenceFrame.getWorldFrame(), registry);

      supportLeg = EnumYoVariable.create("supportLeg", "", RobotSide.class, registry, true);

      // TODO: Have local updatables, instead of adding them to the momentum based controller.
      
      this.updatables.add(new Omega0Updater());
      this.updatables.add(new BipedSupportPolygonsUpdater());
      this.updatables.add(new CapturePointUpdater());
      
//      momentumBasedController.addUpdatable(new Omega0Updater());
//      momentumBasedController.addUpdatable(new BipedSupportPolygonsUpdater());
//      momentumBasedController.addUpdatable(new CapturePointUpdater());
      
      Collection<PlaneContactState> planeContactStates = momentumBasedController.getPlaneContactStates();
      momentumBasedController.addUpdatable(new FootPolygonVisualizer(planeContactStates, dynamicGraphicObjectsListRegistry, registry));

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicPosition capturePointViz = capturePoint.createDynamicGraphicPosition("Capture Point", 0.01, YoAppearance.Blue(),
                                                     GraphicType.ROTATED_CROSS);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("Capture Point", capturePointViz);
         dynamicGraphicObjectsListRegistry.registerArtifact("Capture Point", capturePointViz.createArtifact());
      }
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
      momentumBasedController.initialize();
   }

   // TODO: visibility changed for "public"
   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   protected void setOmega0(double omega0)
   {
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");
      this.omega0.set(omega0);
   }

   protected void computeCapturePoint()
   {
      FramePoint centerOfMass = computeCenterOfMass();
      FrameVector centerOfMassVelocity = computeCenterOfMassVelocity();
      FramePoint2d capturePoint = CapturePointCalculator.computeCapturePoint(centerOfMass, centerOfMassVelocity, getOmega0());
      capturePoint.changeFrame(this.capturePoint.getReferenceFrame());
      this.capturePoint.set(capturePoint.getX(), capturePoint.getY(), 0.0);
   }

   private FramePoint computeCenterOfMass()
   {
      ReferenceFrame centerOfMassFrame = momentumBasedController.getCenterOfMassFrame();

      return new FramePoint(centerOfMassFrame);
   }

   private FrameVector computeCenterOfMassVelocity()
   {
      CenterOfMassJacobian centerOfMassJacobian = momentumBasedController.getCenterOfMassJacobian();

      centerOfMassJacobian.compute();
      FrameVector ret = new FrameVector(ReferenceFrame.getWorldFrame());
      centerOfMassJacobian.packCenterOfMassVelocity(ret);

      return ret;
   }

   protected void updateBipedSupportPolygons(BipedSupportPolygons bipedSupportPolygons)
   {
      SideDependentList<List<FramePoint>> footContactPoints = new SideDependentList<List<FramePoint>>();
      for (RobotSide robotSide : RobotSide.values)
      {
         List<FramePoint> contactPoints = momentumBasedController.getContactPoints(bipedFeet.get(robotSide));
         footContactPoints.put(robotSide, contactPoints);
      }

      bipedSupportPolygons.update(footContactPoints);
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

         setOmega0(omega0Calculator.computeOmega0(cops, admissibleGroundReactionWrench));
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

   public DoubleYoVariable getDesiredCoMHeightAcceleration()
   {
      return desiredCoMHeightAcceleration;
   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public YoFramePoint getCapturePoint()
   {
      return capturePoint;
   }

   public YoFramePoint2d getDesiredICP()
   {
      return desiredICP;
   }
   
   public void getDesiredICP(FramePoint2d pointToPack)
   {
      this.desiredICP.getFramePoint2d(pointToPack);
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
