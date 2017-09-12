package us.ihmc.manipulation.planning.forwaypoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SO3TrajectoryPointCalculatorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final Random random = new Random(1);

   private final double trajectoryTime = 2.0;
   private final double dt = 0.005;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFramePose currentSO3 = new YoFramePose("currentSO3", worldFrame, registry);
   
   private final YoFramePoint currentOrientationViz = new YoFramePoint("currentOrientationViz", worldFrame, registry);
   private final YoFramePoint currentAngularVelocityViz = new YoFramePoint("currentAngularVelocityViz", worldFrame, registry);

   public SO3TrajectoryPointCalculatorVisualizer()
   {
      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);

      double initialTimeLine = -2.0;
      double finalTimeLine = 2.0;
      Point3D currentTimeLinePosition = new Point3D(initialTimeLine, 0.0, 0.5);

      int numberOfWayPoints = 5;

      List<Quaternion> orientations = new ArrayList<Quaternion>();

      for (int i = 0; i < numberOfWayPoints; i++)
      {
         Quaternion orientation = new Quaternion();
         orientation.appendRollRotation(Math.PI * random.nextDouble());
         orientation.appendPitchRotation(Math.PI * random.nextDouble());
         orientation.appendYawRotation(Math.PI * random.nextDouble());

         orientations.add(orientation);
      }

      for (int i = 0; i < numberOfWayPoints; i++)
      {
         YoFramePose orientationViz = new YoFramePose("orientationViz" + i, "viz", worldFrame, registry);
         double timeLine = (finalTimeLine - initialTimeLine) * i / (numberOfWayPoints - 1);
         Point3D timeLinePosition = new Point3D(timeLine, 0.0, 0.5);

         orientationViz.setPosition(timeLinePosition);
         orientationViz.setOrientation(orientations.get(i));

         yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("waypointViz" + i, orientationViz, 0.3));
      }

      yoGraphicsListRegistry.registerYoGraphic("viz", new YoGraphicCoordinateSystem("currentSO3", currentSO3, 0.5, YoAppearance.Red()));
      yoGraphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("currentOrientationViz", currentOrientationViz, 0.05, YoAppearance.Red()));
      yoGraphicsListRegistry.registerYoGraphic("viz",
                                               new YoGraphicPosition("currentAngularVelocityViz", currentAngularVelocityViz, 0.05, YoAppearance.Black()));

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      /*
       * calculate angular velocities.
       */
      SO3TrajectoryPointCalculator calculator = new SO3TrajectoryPointCalculator();
      calculator.setFirstTrajectoryPointTime(0.0);
      for (int i = 0; i < numberOfWayPoints; i++)
      {
         calculator.appendTrajectoryPointOrientation(i * trajectoryTime / (numberOfWayPoints - 1), orientations.get(i));
      }
      calculator.compute();

      for (double t = 0.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);

         currentTimeLinePosition.setX((finalTimeLine - initialTimeLine) * t / trajectoryTime);
         currentSO3.setPosition(currentTimeLinePosition);
         currentSO3.setOrientation(calculator.getOrientation(t));

         
         Vector3D currentOrientation = new Vector3D();
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(calculator.getOrientation(t), currentOrientation);                 
         currentOrientationViz.set(currentOrientation);         
//         currentAngularVelocityViz.set(numerical angular vs );
         
         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();

   }

   public static void main(String[] args)
   {
      new SO3TrajectoryPointCalculatorVisualizer();
   }
}
