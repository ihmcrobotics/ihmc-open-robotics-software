package us.ihmc.darpaRoboticsChallenge.calib;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.functions.FunctionNtoM;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.api.AtlasJointId;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePose;

public class AtlasKinematicCalibrator
{
   private final SDFRobot robot;
   protected final SDFFullRobotModel fullRobotModel;
   
   protected final OneDoFJoint[] joints;
   protected final ArrayList<Map<String, Object>> q = new ArrayList<>();
   protected final ArrayList<Map<String, Object>> qout = new ArrayList<>();
   private SDFFullRobotModelVisualizer visualizer=null;
   final static int RESIDUAL_DOF = 6;
   final static boolean DEBUG=false;
  
   protected final YoVariableRegistry registry;
   protected SimulationConstructionSet scs;
   IntegerYoVariable yoIndex;
 

   public AtlasKinematicCalibrator()
   {
      //load robot
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_NO_HANDS_ADDED_MASS, false);
      JaxbSDFLoader robotLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      robot = robotLoader.createRobot(jointMap, false);
      registry= robot.getRobotsYoVariableRegistry();
      fullRobotModel = robotLoader.createFullRobotModel(jointMap);
      joints = fullRobotModel.getOneDoFJoints();
      yoIndex =  new IntegerYoVariable("index", registry);

   }
   
   /**
    * do not override, use addDynamicGraphicObjects
    */
   protected void createDisplay()
   {
      createDisplay(8192);
   }

   protected void createDisplay(int bufferSize)
   {
      visualizer = new SDFFullRobotModelVisualizer(robot, 1, 0.01); //100hz sample rate
      visualizer.setFullRobotModel(fullRobotModel);

      scs = new SimulationConstructionSet(robot,bufferSize);
      scs.setGroundVisible(false);
      visualizer.registerSCS(scs);
      addDynamicGraphicObjects();

      scs.startOnAThread();
      
   }
   
   
   protected void addDynamicGraphicObjects()
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
      for(int i=0;i < maxIter;i++)
      {
         converged = optimizer.iterate();
         System.out.println("iter " + i + " obj: " + optimizer.getFunctionValue() + " converged:" + converged);         
         if(optimizer.isConverged())
            break;
      }
      System.out.println("prmChg"+prm[0]+" "+ optimizer.getParameters()[0]);

      System.arraycopy(optimizer.getParameters(), 0, prm, 0, prm.length);
      System.out.println("Optimiztion finished.");
   }
   
 


}
  

