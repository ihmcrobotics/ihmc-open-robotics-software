package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CaptureRegion1DMultiStepVisualizer
{
   private static final double moveDuration = 3.0;
   private static final double maxStepLength = 1.0;
   private static final double stepDuration = 0.5;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();


   public CaptureRegion1DMultiStepVisualizer()
   {
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      FramePoint3D initialPosition = new FramePoint3D(worldFrame, 0.0, 0.0, 1.0);
      double velocity = getMaxOneStepVelocity();
      FrameVector3D initialVelocity1 = new FrameVector3D(worldFrame, velocity, 0.0, 0.0);

      PointMassRobot robot1 = new PointMassRobot("1", initialPosition, initialVelocity1);
      PointMassRobot robot2 = new PointMassRobot("2", initialPosition, initialVelocity1);
      PointMassRobot robot3 = new PointMassRobot("3", initialPosition, initialVelocity1);
      robot1.setController(new Simple2DStepController("robot", "1", robot1, stepDuration, () -> maxStepLength, graphicsListRegistry));
      robot2.setController(new Simple2DStepController("robot", "2", robot2, stepDuration, () -> getStepLengthForTwoStepStop(), graphicsListRegistry));
      robot3.setController(new Simple2DStepController("robot", "3", robot3, stepDuration, () -> getStepLengthForThreeStepStop(), graphicsListRegistry));

      robot2.setController(new GraphicsController(graphicsListRegistry));

      double dt = 0.001;
      SimulationConstructionSet scs = new SimulationConstructionSet();
      scs.addRobot(robot1);
      scs.addRobot(robot2);
      scs.addRobot(robot3);
      scs.setDT(dt, 1);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      scs.simulate(moveDuration);
//      scs.stop();
   }

   private static double getMaxOneStepVelocity()
   {
      double omega = Math.sqrt(9.81);
      return maxStepLength / Math.exp(omega * stepDuration) * omega;
   }

   private static double getStepLengthForTwoStepStop()
   {
      double omega = Math.sqrt(9.81);
      double exp = Math.exp(omega * stepDuration);
      return exp * exp * getMaxOneStepVelocity() / (1 + exp) / omega;
   }

   private static double getStepLengthForThreeStepStop()
   {
      double omega = Math.sqrt(9.81);
      double exp = Math.exp(omega * stepDuration);
      return exp * exp * exp * getMaxOneStepVelocity() / (1 + exp * exp + exp) / omega;
   }

   private static class GraphicsController implements RobotController
   {
      private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      private final YoGraphicLineSegment oneStepRegion;
      private final YoGraphicLineSegment twoStepRegion;
      private final YoGraphicLineSegment threeStepRegion;

      public GraphicsController(YoGraphicsListRegistry graphicsListRegistry)
      {
         AppearanceDefinition oneStep = YoAppearance.Yellow();
         AppearanceDefinition twoStep = YoAppearance.Yellow();
         AppearanceDefinition threeStep = YoAppearance.Yellow();
         twoStep.setTransparency(0.5);
         threeStep.setTransparency(0.75);
         oneStepRegion = new YoGraphicLineSegment("oneStep", "CaptureRegion", worldFrame, oneStep, registry);
         twoStepRegion = new YoGraphicLineSegment("twoStep", "CaptureRegion", worldFrame, twoStep, registry);
         threeStepRegion = new YoGraphicLineSegment("threeStep", "CaptureRegion", worldFrame, threeStep, registry);

         graphicsListRegistry.registerYoGraphic("graphics", oneStepRegion);
         graphicsListRegistry.registerYoGraphic("graphics", twoStepRegion);
         graphicsListRegistry.registerYoGraphic("graphics", threeStepRegion);
      }

      @Override
      public void doControl()
      {
         double omega = Math.sqrt(9.81);
         double exp = Math.exp(-omega * stepDuration);
         oneStepRegion.setStartAndEnd(new Point3D(maxStepLength - 0.01, 0.0, 0.02), new Point3D(maxStepLength + 0.01, 0.0, 0.02));
         double twoStepExtra = exp * maxStepLength;
         double threeStepExtra = exp * exp * maxStepLength;
         twoStepRegion.setStartAndEnd(new Point3D(maxStepLength - twoStepExtra, 0.0, 0.02), new Point3D(maxStepLength + twoStepExtra, 0.0, 0.02));
         threeStepRegion.setStartAndEnd(new Point3D(maxStepLength - twoStepExtra - threeStepExtra, 0.0, 0.02), new Point3D(maxStepLength + twoStepExtra + threeStepExtra, 0.0, 0.02));
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
      private final DoubleProvider stepLengthProvider;

      public Simple2DStepController(String robotPrefix, String robotPostFix, PointMassRobot robot, double timeToTakeAStep, DoubleProvider stepLengthProvider,
                                    YoGraphicsListRegistry graphicsListRegistry)
      {
         this.robot = robot;
         this.stepLengthProvider = stepLengthProvider;

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
         YoGraphicPosition icpVisualizer = new YoGraphicPosition("currentICP" + robotPostFix, currentICP, size, YoAppearance.Blue());
         YoGraphicVector grfVisualizer = new YoGraphicVector("desiredGRF" + robotPostFix, desiredCMP, desiredForce, 0.1, YoAppearance.Red());
         YoGraphicPosition cmpVisualizer = new YoGraphicPosition("desiredCMP" + robotPostFix, desiredCMP, size, YoAppearance.Green());

         String name = robotPrefix + " VIz " + robotPostFix;
         graphicsListRegistry.registerYoGraphic(name, dcmVisualizer);
         graphicsListRegistry.registerYoGraphic(name, icpVisualizer);
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
            double maxX = desiredCMP.getX() + stepLengthProvider.getValue();
            desiredCMP.set(currentICP);
            desiredCMP.setX(Math.min(maxX, currentICP.getX()));
            timeOfLastStep.set(robot.getTime());
         }

         currentICP.setZ(0.02);

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
      new CaptureRegion1DMultiStepVisualizer();
   }
}
