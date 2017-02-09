package us.ihmc.atlas.calib;

import java.util.ArrayList;
import java.util.Map;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.functions.FunctionNtoM;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationToolkit.visualizers.FullRobotModelVisualizer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class AtlasKinematicCalibrator
{
   private final FloatingRootJointRobot robot;
   protected final FullHumanoidRobotModel fullRobotModel;

   protected final OneDoFJoint[] joints;
   protected final ArrayList<Map<String, Double>> q = new ArrayList<>();
   protected final ArrayList<Map<String, Double>> qout = new ArrayList<>();
   private FullRobotModelVisualizer visualizer = null;
   final static int RESIDUAL_DOF = 6;
   final static boolean DEBUG = false;

   protected final YoVariableRegistry registry;
   protected SimulationConstructionSet scs;
   IntegerYoVariable yoIndex;


   public AtlasKinematicCalibrator(DRCRobotModel robotModel)
   {
      //load robot
      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      registry = robot.getRobotsYoVariableRegistry();
      fullRobotModel = robotModel.createFullRobotModel();
      joints = fullRobotModel.getOneDoFJoints();

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
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(bufferSize);
      scs = new SimulationConstructionSet(robot, parameters);
      visualizer = new FullRobotModelVisualizer(scs, fullRobotModel, 0.01); //100hz sample rate
      
      scs.setGroundVisible(false);
      setupDynamicGraphicObjects();

      scs.startOnAThread();
      scs.maximizeMainWindow();
      
      yoIndex = new IntegerYoVariable("index", visualizer.getRobotRegistry());
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
  

