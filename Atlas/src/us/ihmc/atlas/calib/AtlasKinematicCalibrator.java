package us.ihmc.atlas.calib;

import java.util.ArrayList;
import java.util.Map;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.functions.FunctionNtoM;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AtlasKinematicCalibrator
{
   private final SDFRobot robot;
   protected final SDFFullRobotModel fullRobotModel;

   protected final OneDoFJoint[] joints;
   protected final ArrayList<Map<String, Double>> q = new ArrayList<>();
   protected final ArrayList<Map<String, Double>> qout = new ArrayList<>();
   private SDFFullRobotModelVisualizer visualizer = null;
   final static int RESIDUAL_DOF = 6;
   final static boolean DEBUG = false;

   protected final YoVariableRegistry registry;
   protected SimulationConstructionSet scs;
   IntegerYoVariable yoIndex;


   public AtlasKinematicCalibrator(DRCRobotModel robotModel)
   {
      //load robot
      robot = robotModel.createSdfRobot(false);
      registry = robot.getRobotsYoVariableRegistry();
      fullRobotModel = robotModel.createFullRobotModel();
      joints = fullRobotModel.getOneDoFJoints();
      yoIndex = new IntegerYoVariable("index", registry);

   }

   /**
    * ido not override, use addDynamicGraphicObjects
    */
   protected void createDisplay()
   {
      createDisplay(8192);
   }

   protected void createDisplay(int bufferSize)
   {
      visualizer = new SDFFullRobotModelVisualizer(robot, 1, 0.01); //100hz sample rate
      visualizer.setMainRegistry(registry, fullRobotModel, null);

      scs = new SimulationConstructionSet(robot, bufferSize);
      scs.setGroundVisible(false);
      visualizer.registerSCS(scs);
      setupDynamicGraphicObjects();

      scs.startOnAThread();
      scs.maximizeMainWindow();
   }


   protected void setupDynamicGraphicObjects()
   {

   }

   /**
    * do not override, use updateDynamicGraphicObjects
    */
   protected void displayUpdate(int index)
   {
      yoIndex.set(index);
      updateDynamicGraphicsObjects(index);
      visualizer.update(1);
   }

   protected void updateDynamicGraphicsObjects(int index)
   {

   }



   public void calibrate(FunctionNtoM residualFunc, double[] prm, int maxIter)
   {
      UnconstrainedLeastSquares optimizer = FactoryOptimization.leastSquaresLM(1e-3, true);
      optimizer.setFunction(residualFunc, null);
      optimizer.initialize(prm, 1e-12, 1e-12);
      boolean converged;
      for (int i = 0; i < maxIter; i++)
      {
         converged = optimizer.iterate();
         System.out.println("iter " + i + " obj: " + optimizer.getFunctionValue() + " converged:" + converged);
         if (optimizer.isConverged())
            break;
      }
      System.out.println("prmChg" + prm[0] + " " + optimizer.getParameters()[0]);

      System.arraycopy(optimizer.getParameters(), 0, prm, 0, prm.length);
      System.out.println("Optimiztion finished.");
   }

}
  

