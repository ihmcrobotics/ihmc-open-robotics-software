package us.ihmc.wholeBodyController;

import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public abstract class WholeBodyTrajectoryTest
{
   static private SDFFullRobotModel actualRobotModel;
   static private SDFFullRobotModel desiredRobotModel;
   static private WholeBodyIkSolver hikSolver;
   
   static private final  YoVariableRegistry registry = new YoVariableRegistry("WholeBodyIkSolverTestFactory_Registry"); 
    
   private final ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handArrayList = new ArrayList<Pair<ReferenceFrame, ReferenceFrame>>();

   static private final boolean DEBUG = false && !BambooTools.isRunningOnBamboo();
   

   public abstract WholeBodyControllerParameters getRobotModel();
   public abstract FullRobotModelVisualizer getFullRobotModelVisualizer();
   public abstract SimulationConstructionSet getSimulationConstructionSet();
   

   public WholeBodyTrajectoryTest()
   {
      if( actualRobotModel== null)
      {
         actualRobotModel = getRobotModel().createFullRobotModel();
         desiredRobotModel = getRobotModel().createFullRobotModel();
   
         try{
            hikSolver = new WholeBodyIkSolver(getRobotModel(), actualRobotModel);
         }
         catch (IOException e) {
            e.printStackTrace();
         }   
         
         Vector3d rootPosition = new Vector3d(0,0, 0.93);
         actualRobotModel.getRootJoint().setPosition( rootPosition);   
      }    
   }
   

 
}
