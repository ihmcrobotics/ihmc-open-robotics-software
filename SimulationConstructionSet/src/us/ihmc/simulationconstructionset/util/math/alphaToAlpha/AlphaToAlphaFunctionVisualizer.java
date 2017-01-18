package us.ihmc.simulationconstructionset.util.math.alphaToAlpha;

import java.util.ArrayList;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Plane;
import us.ihmc.robotics.alphaToAlpha.AlphaToAlphaFunction;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * <p>VisualizeAlphaToAlphaFunction </p>
 *
 * <p>For visualizing an AlphaToAlphaFunction </p>
 *
 * <p>Copyright (c) 2007</p>
 *
 * <p>Company: IHMC</p>
 *
 * @author IHMC LearningLocomotion Team
 * @version 1.0
 */
public class AlphaToAlphaFunctionVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("AlphaToAlpha");
   private final DoubleYoVariable alphaPrime = new DoubleYoVariable("alphaPrime", registry);
   private final DoubleYoVariable alpha = new DoubleYoVariable("alpha", registry);

   public AlphaToAlphaFunctionVisualizer(AlphaToAlphaFunction alphaToAlphaFunction)
   {
      BallRobot ballRobot = new BallRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(ballRobot);
      scs.addVarList(registry.createVarList());

      Thread thread = new Thread(scs);
      thread.start();

      ArrayList<Double> alphas = new ArrayList<Double>();
      ArrayList<Double> alphaPrimes = new ArrayList<Double>();

      for (alpha.set(0.0); alpha.getDoubleValue() < 1.0; alpha.set(alpha.getDoubleValue() + 0.001))
      {
         alphaPrime.set(alphaToAlphaFunction.getAlphaPrime(alpha.getDoubleValue()));

         alphas.add(alpha.getDoubleValue());
         alphaPrimes.add(alphaPrime.getDoubleValue());

         ballRobot.setXY(alphaPrime.getDoubleValue(), 0.0);

         scs.tickAndUpdate();
      }

      throw new RuntimeException("Plotting not available");

      // PlotterForPoints.plotGraphXversusY(alphas, alphaPrimes); // This is inside the LittleDog project. Maybe this should be moved to SimulationConstructionSetUtilities
   }

   private static class BallRobot extends Robot
   {
      private final FloatingPlanarJoint planarJoint;
      private final DoubleYoVariable x, y;

      public BallRobot()
      {
         super("Ball");

         planarJoint = new FloatingPlanarJoint("planar", this, Plane.XY);
         Link ball = new Link("ball");
         Graphics3DObject linkGraphics = new Graphics3DObject();
         ball.setMass(1.0);
         ball.setMomentOfInertia(0.1, 0.1, 0.1);
         linkGraphics.translate(0.0, 0.0, 0.1);
         linkGraphics.addSphere(0.1, YoAppearance.Red());
         ball.setLinkGraphics(linkGraphics);
         planarJoint.setLink(ball);

         this.addRootJoint(planarJoint);

         x = (DoubleYoVariable)this.getVariable("q_x");
         y = (DoubleYoVariable)this.getVariable("q_y");

      }

      public void setXY(double x, double y)
      {
         this.x.set(x);
         this.y.set(y);
      }
   }
}
