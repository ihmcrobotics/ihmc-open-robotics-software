package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CaptureRegion2DDifferentDurationsVisualizer
{
   private static final double moveDuration = 3.0;
   private static final double maxStepLength = 1.25;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();


   public CaptureRegion2DDifferentDurationsVisualizer()
   {
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      FramePoint3D initialPosition = new FramePoint3D(worldFrame, 0.022, 0.0, 1.0);
      FrameVector3D initialVelocity = new FrameVector3D(worldFrame, 0.1, 0.0, 0.0);

      PointMassRobot robot1 = new PointMassRobot("1", initialPosition, initialVelocity);
      PointMassRobot robot2 = new PointMassRobot("2", initialPosition, initialVelocity);
      PointMassRobot robot3 = new PointMassRobot("3", initialPosition, initialVelocity);
      PointMassRobot robot4 = new PointMassRobot("4", initialPosition, initialVelocity);
      PointMassRobot robot5 = new PointMassRobot("5", initialPosition, initialVelocity);
      robot1.setController(new Simple2DStepController("robot", "1", robot1, 0.5, graphicsListRegistry));
      robot2.setController(new Simple2DStepController("robot", "2", robot2, 0.625, graphicsListRegistry));
      robot3.setController(new Simple2DStepController("robot", "3", robot3, 0.75, graphicsListRegistry));
      robot4.setController(new Simple2DStepController("robot", "4", robot4, 0.875, graphicsListRegistry));
      robot5.setController(new Simple2DStepController("robot", "5", robot5, 1.0, graphicsListRegistry));

      double dt = 0.001;
      SimulationConstructionSet scs = new SimulationConstructionSet(robot1);
      scs.addRobot(robot2);
      scs.addRobot(robot3);
      scs.addRobot(robot4);
      scs.addRobot(robot5);
      scs.setDT(dt, 1);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      scs.simulate(moveDuration);
//      scs.stop();
   }

   private static class Simple2DStepController implements RobotController
   {
      private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      private final PointMassRobot robot;

      private final YoFramePoint3D currentCoM;
      private final YoFrameVector3D currentCoMVelocity;
      private final YoFramePoint3D currentDCM;
      private final YoFramePoint3D currentICP;
      private final YoFramePoint3D desiredCMP;
      private final YoFrameVector3D desiredForce;
      private final YoDouble stepDuration;
      private final YoDouble timeOfLastStep;

      public Simple2DStepController(String robotPrefix, String robotPostFix, PointMassRobot robot, double timeToTakeAStep,
                                    YoGraphicsListRegistry graphicsListRegistry)
      {
         this.robot = robot;

         currentCoM = new YoFramePoint3D("currentCoM" + robotPostFix, worldFrame, registry);
         currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity" + robotPostFix, worldFrame, registry);
         currentDCM = new YoFramePoint3D("currentDCM" + robotPostFix, worldFrame, registry);
         currentICP = new YoFramePoint3D("currentICP" + robotPostFix, worldFrame, registry);
         desiredCMP = new YoFramePoint3D("desiredCMP" + robotPostFix, worldFrame, registry);
         desiredForce = new YoFrameVector3D("desiredForce" + robotPostFix, worldFrame, registry);
         stepDuration = new YoDouble("stepDuration" + robotPostFix, registry);
         timeOfLastStep = new YoDouble("timeOfLastStep" + robotPostFix, registry);

         stepDuration.set(timeToTakeAStep);

         double size = 0.015;
         YoGraphicPosition dcmVisualizer = new YoGraphicPosition("currentDCM" + robotPostFix, currentDCM, size, YoAppearance.Blue());
         YoGraphicVector grfVisualizer = new YoGraphicVector("desiredGRF" + robotPostFix, desiredCMP, desiredForce, 0.1, YoAppearance.Red());
         YoGraphicPosition cmpVisualizer = new YoGraphicPosition("desiredCMP" + robotPostFix, desiredCMP, size, YoAppearance.Green());

         String name = robotPrefix + " VIz " + robotPostFix;
         graphicsListRegistry.registerYoGraphic(name, dcmVisualizer);
         graphicsListRegistry.registerYoGraphic(name, grfVisualizer);
         graphicsListRegistry.registerYoGraphic(name, cmpVisualizer);
      }


      @Override
      public void doControl()
      {
         currentCoM.set(robot.getFloatingJoint().getPosition());
         currentCoMVelocity.set(robot.getFloatingJoint().getLinearVelocity());

         double omega = Math.sqrt(9.81 / currentCoM.getZ());
         CapturePointTools.computeCapturePointPosition(currentCoM, currentCoMVelocity, omega, currentDCM);
         currentICP.set(currentDCM);
         currentICP.setZ(0.0);

         if (robot.getTime() > timeOfLastStep.getValue() + stepDuration.getValue())
         {
            double maxX = desiredCMP.getX() + maxStepLength;
            desiredCMP.set(currentICP);
            desiredCMP.setX(Math.min(maxX, currentICP.getX()));
            timeOfLastStep.set(robot.getTime());
         }

         currentICP.setZ(0.01);

         desiredForce.sub(currentCoM, desiredCMP);
         desiredForce.scale(robot.getMass() * omega * omega);

         robot.getAllExternalForcePoints().get(0).setForce(desiredForce);
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }
   }



   private static class PointMassRobot extends Robot
   {
      private static final double RadiusScale = 1.0;
      private static final double M1 = 1.7;
      private static final double Ixx1 = 0.1, Iyy1 = 0.1, Izz1 = 0.1;

      private final FloatingJoint floatingJoint;
      private final Link robotLink;

      public PointMassRobot(String postfix, FramePoint3D initialPosition, FrameVector3D initialVelocity)
      {
         super("SRBRobot" + postfix);

         floatingJoint = new FloatingJoint("base", new Vector3D(0.0, 0.0, 0.0), this);
         robotLink = base("Base", YoAppearance.Green());
         floatingJoint.setLink(robotLink);

         floatingJoint.getPosition().set(initialPosition);
         floatingJoint.getLinearVelocity().set(initialVelocity);

         ExternalForcePoint externalForcePoint1 = new ExternalForcePoint("ForcePoint", this);
         floatingJoint.addExternalForcePoint(externalForcePoint1);

         addRootJoint(floatingJoint);
      }

      public double getMass()
      {
         return M1;
      }

      public FloatingJoint getFloatingJoint()
      {
         return floatingJoint;
      }

      public Link getLink()
      {
         return robotLink;
      }

      private static Link base(String name, AppearanceDefinition appearance)
      {
         Link ret = new Link(name);
         ret.setMass(M1);
         ret.setMomentOfInertia(Ixx1, Iyy1, Izz1);
         ret.setComOffset(0.0, 0.0, 0.0);

         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addEllipsoid(RadiusScale * Ixx1, RadiusScale * Iyy1, RadiusScale * Izz1, appearance);

         ret.setLinkGraphics(linkGraphics);

         return ret;
      }
   }

   public static void main(String[] args)
   {
      new CaptureRegion2DDifferentDurationsVisualizer();
   }
}
