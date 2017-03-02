package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.MockQuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;

public class QuadrupedPathPreviewVisualizer implements RobotController
{
   private static final double simulateDT = 0.01;
   private static final int recordFrequency = 1;

   private final YoVariableRegistry registry = new YoVariableRegistry(getName());
   private final SimulationConstructionSet scs;
   private final Robot robot;
   private final FloatingJoint rootJoint;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final MockQuadrupedReferenceFrames referenceFrames = new MockQuadrupedReferenceFrames();
   private final EnumYoVariable<RobotQuadrant> swingLeg = new EnumYoVariable<RobotQuadrant>("swingLeg", registry, RobotQuadrant.class, true);
   private final QuadrupedPathPreview pathPreview;
   private final YoFrameVector desiredVelocity = new YoFrameVector("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable desiredYawRate = new DoubleYoVariable("desiredYawRate", registry);
   private final QuadrantDependentList<YoFramePoint> yoFootPositions = new QuadrantDependentList< YoFramePoint>();
   
   
   public QuadrupedPathPreviewVisualizer()
   {
      scs = new SimulationConstructionSet();
      
      desiredVelocity.setX(0.24);

      robot = new Robot("viz");
      rootJoint = new FloatingJoint("floating", new Vector3D(), robot);
      robot.getRobotsYoVariableRegistry();
      robot.setController(this);
      scs.setRobot(robot);
      swingLeg.set(RobotQuadrant.FRONT_RIGHT);
      
      
      DefaultSwingTargetGeneratorParameters defaultFootStepParameters = new DefaultSwingTargetGeneratorParameters();
      MidFootZUpSwingTargetGenerator swingTargetGenerator = new MidFootZUpSwingTargetGenerator(defaultFootStepParameters, referenceFrames, registry);
      
      pathPreview = new QuadrupedPathPreview(swingTargetGenerator, referenceFrames, registry, yoGraphicsListRegistry);
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         YoFramePoint footPosition = new YoFramePoint(prefix + "footPosition", ReferenceFrame.getWorldFrame(), registry);
         yoFootPositions.set(robotQuadrant, footPosition);
      }
      
      yoFootPositions.get(RobotQuadrant.FRONT_LEFT).set(new Vector3D(0.12, 0.14, 0.0));
      yoFootPositions.get(RobotQuadrant.HIND_LEFT).set(new Vector3D(-0.12, 0.14, 0.0));

      yoFootPositions.get(RobotQuadrant.FRONT_RIGHT).set(new Vector3D(0.12, -0.14, 0.0));
      yoFootPositions.get(RobotQuadrant.HIND_RIGHT).set(new Vector3D(-0.12, -0.14, 0.0));
      
      boolean showOverheadView = true;
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(showOverheadView);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(simulateDT, recordFrequency);
      scs.startOnAThread();
      scs.simulate();
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getClass().getName();
   }

   @Override
   public void doControl()
   {
      referenceFrames.update(yoFootPositions);
      pathPreview.update(swingLeg.getEnumValue(), desiredVelocity.getFrameVectorCopy(), desiredYawRate.getDoubleValue());
   }
   
   public static void main(String[] args)
   {
     new QuadrupedPathPreviewVisualizer();
   }
}
