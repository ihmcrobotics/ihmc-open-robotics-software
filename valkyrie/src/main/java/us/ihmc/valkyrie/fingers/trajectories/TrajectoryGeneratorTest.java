package us.ihmc.valkyrie.fingers.trajectories;

import static org.junit.Assert.assertFalse;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TrajectoryGeneratorTest
{
   enum TrajectoryType
   {
      Linear, Sinusoidal
   }

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double wholeTime = 20.0;
   private final double dt = 0.05;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (wholeTime / dt / recordFrequency + 2);

   private final YoDouble yoTime;
   private final YoDouble yoQ = new YoDouble("yoQ", registry);
   private final YoDouble yoQd = new YoDouble("yoQd", registry);

   private final TrajectoryGenerator trajectoryGenerator;
   private final TrajectoryInterface trajectory;

   private final TrajectoryType testTrajectoryType = TrajectoryType.Sinusoidal;

   public TrajectoryGeneratorTest()
   {
      System.out.println("Hello");

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");
      yoTime = new YoDouble("yoTime", registry);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      yoQ.set(1.1);
      yoQd.set(0.0);
      switch (testTrajectoryType)
      {
      case Linear:
         trajectory = new LinearTrajectory(yoQ.getDoubleValue());
         break;
      case Sinusoidal:
         trajectory = new SinusoidalTrajectory(yoQ.getDoubleValue());
         break;
      default:
         trajectory = null;
         assertFalse(true);
         break;
      }
      trajectoryGenerator = new TrajectoryGenerator("aa", yoTime, registry, trajectory);

      boolean executed1 = false;
      boolean executed2 = false;

      for (double t = 0.0; t <= wholeTime; t += dt)
      {
         yoTime.add(dt);
         trajectoryGenerator.doControl();
         yoQ.set(trajectoryGenerator.getDesiredQ());
         yoQd.set(trajectoryGenerator.getDesiredQd());

         if (t > 1.0 && !executed1)
         {
            trajectoryGenerator.executeTrajectory(3.0, 0.5, 5.0);
            executed1 = true;
         }

         if (t > 10.0 && !executed2)
         {
            trajectoryGenerator.executeTrajectory(2.0, 1.0, 10.0);
            executed2 = true;
         }

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();

      System.out.println("Bye");
   }

   public static void main(String[] args)
   {
      new TrajectoryGeneratorTest();
   }
}