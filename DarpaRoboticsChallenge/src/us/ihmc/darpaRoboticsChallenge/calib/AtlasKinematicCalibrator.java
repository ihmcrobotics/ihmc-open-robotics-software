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

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePose;

public class AtlasKinematicCalibrator
{
   private final SDFRobot robot;
   private final SDFFullRobotModel fullRobotModel;
   
   private final OneDoFJoint[] joints;
   private final ArrayList<Map<String, Double>> q = new ArrayList<>();
   private final ArrayList<Map<String, Double>> qout = new ArrayList<>();
   private SDFFullRobotModelVisualizer visualizer=null;
   final static int RESIDUAL_DOF = 6;
   final static boolean DEBUG=false;
  
   //YoVariables for Display
   private final YoVariableRegistry registry;
   private final YoFramePoint ypLeftEE, ypRightEE;
   private final YoFramePose yposeLeftEE, yposeRightEE;
  

   public AtlasKinematicCalibrator()
   {
      //load robot
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_NO_HANDS_ADDED_MASS, false);
      JaxbSDFLoader robotLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      robot = robotLoader.createRobot(jointMap, false);
      registry= robot.getRobotsYoVariableRegistry();
      fullRobotModel = robotLoader.createFullRobotModel(jointMap);
      joints = fullRobotModel.getOneDoFJoints();
      
      //yo*
      ypLeftEE = new YoFramePoint("leftEE", ReferenceFrame.getWorldFrame(), registry);
      ypRightEE = new YoFramePoint("rightEE", ReferenceFrame.getWorldFrame(),registry);
      yposeLeftEE = new YoFramePose("leftPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
      yposeRightEE = new YoFramePose("rightPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
   }
   
   private void createDisplay()
   {
      visualizer = new SDFFullRobotModelVisualizer(robot, 1, 0.01); //100hz sample rate
      visualizer.setFullRobotModel(fullRobotModel);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot,32768);
      scs.setGroundVisible(false);
      visualizer.registerSCS(scs);
      
      double transparency = 0.5;
      double scale=0.02;
      DynamicGraphicPosition dgpLeftEE = new DynamicGraphicPosition("dgpLeftEE", ypLeftEE, scale, new YoAppearanceRGBColor(Color.BLUE, transparency));
      DynamicGraphicPosition dgpRightEE = new DynamicGraphicPosition("dgpRightEE", ypRightEE, scale, new YoAppearanceRGBColor(Color.RED, transparency));
      
      scs.addDynamicGraphicObject(dgpLeftEE);
      scs.addDynamicGraphicObject(dgpRightEE);

      DynamicGraphicCoordinateSystem dgPoseLeftEE = new DynamicGraphicCoordinateSystem("dgposeLeftEE", yposeLeftEE, 5*scale);
      DynamicGraphicCoordinateSystem dgPoseRightEE = new DynamicGraphicCoordinateSystem("dgposeRightEE", yposeRightEE, 5*scale);
      scs.addDynamicGraphicObject(dgPoseLeftEE);
      scs.addDynamicGraphicObject(dgPoseRightEE);
      
      
      scs.startOnAThread();
      
   }

   public ArrayList<OneDoFJoint> getHeadArmJoints(RobotSide side)
   {
      throw new RuntimeException("not implemented");
   }
   
   public ArrayList<OneDoFJoint> getArmJoints()
   {
      ArrayList<OneDoFJoint> armJoints = new ArrayList<OneDoFJoint>();
      for(int i=0;i<joints.length;i++)
      {
         if(joints[i].getName().matches(".*arm.*"))
         {
            armJoints.add(joints[i]);
            System.out.println("arm "+ i + " "+joints[i].getName());
         }
         
      }
      return armJoints;
   }

   public void loadJointAnglesFromFile()

   
   {
      String calib_file = "data/coupledWristLog_20131204";
      BufferedReader reader = null;
      try
      {
         reader = new BufferedReader(new FileReader(calib_file));
      }
      catch (FileNotFoundException e1)
      {
         System.out.println("Cannot load calibration file " + calib_file);
         e1.printStackTrace();
      }

      String line;
      final int numJoints = 28;
      System.out.println("total joints should be " + numJoints);
      try
      {
         while ((line = reader.readLine()) != null)
         {
            if (line.matches("^entry.*"))
            {
               Map<String, Double> q_ = new HashMap<>();
               Map<String, Double> qout_ = new HashMap<>();

               for (int i = 0; i < numJoints; i++)
               {
                  line = reader.readLine();
                  if (line != null)
                  {
                     String[] items = line.split("\\s");
                     q_.put(items[0], new Double(items[1]));
                     qout_.put(items[0], new Double(items[2]));
                  }
                  else
                  {
                     System.out.println("One ill-formed data entry");
                     break;
                  }

               }

               if (q_.size() == numJoints)
                  q.add(q_);
               if (qout_.size() == numJoints)
                  qout.add(qout_);
            }
         }
      }
      catch (IOException e1)
      {
         System.err.println("File reading error");
         e1.printStackTrace();
      }
      System.out.println("total entry loaded q/qout " + q.size() + "/" + qout.size());
   }
   
   
   public void displayUpdate()
   {
      FramePoint
      leftEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM)  ,0, 0.13,0),
      rightEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM),0,-0.13,0);
      

      ypLeftEE.set(leftEE.changeFrameCopy(CalibUtil.world));
      ypRightEE.set(rightEE.changeFrameCopy(CalibUtil.world));
      
      yposeLeftEE.set(new FramePose(leftEE, new FrameOrientation(leftEE.getReferenceFrame())).changeFrameCopy(CalibUtil.world));
      yposeRightEE.set(new FramePose(rightEE,new FrameOrientation(rightEE.getReferenceFrame())).changeFrameCopy(CalibUtil.world));

      visualizer.update(1);
   }

   

   
   public KinematicCalibrationResidual getArmLoopResidualObject()
   {
      ArrayList<String> calJointNames = CalibUtil.toStringArrayList(getArmJoints());
      return new KinematicCalibrationResidual(fullRobotModel, calJointNames, q);
   }
   
   public KinematicCalibrationResidual getCameraArmResidualObject()
   {
      throw new RuntimeException("Not Implemetned ");
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
   
   public static void main(String[] arg) throws InterruptedException
   {
      AtlasKinematicCalibrator calib = new AtlasKinematicCalibrator();
      calib.loadJointAnglesFromFile();
      YoFramePose yoResidual0 = new YoFramePose("residual0", "", ReferenceFrame.getWorldFrame(),calib.registry);
      YoFramePose yoResidual = new YoFramePose("residual", "",ReferenceFrame.getWorldFrame(),calib.registry);

      // calJointNames order is the prm order
      KinematicCalibrationResidual residualFunc =calib.getArmLoopResidualObject();
      double[] prm = new double[residualFunc.getN()];
      double[] residual0 = residualFunc.calcResiduals(prm);
      calib.calibrate(residualFunc,prm, 100);
      double[] residual = residualFunc.calcResiduals(prm);
      

      //display prm in readable format
      Map<String,Double> qoffset= residualFunc.prmArrayToJointMap(prm);
      for(String jointName: qoffset.keySet())
      {
         System.out.println(jointName + " "+ qoffset.get(jointName));
      }
      System.out.println("wristSpacing "+prm[prm.length-1]);
      
      //push data to visualizer
      boolean start_scs=false;
      if(start_scs)
      {
         calib.createDisplay();
         
         for(int i=0;i<calib.q.size();i++)
         {
            CalibUtil.setRobotModelFromData(calib.fullRobotModel, CalibUtil.addQ(calib.q.get(i),qoffset));
            yoResidual0.setXYZYawPitchRoll(Arrays.copyOfRange(residual0, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));
            yoResidual.setXYZYawPitchRoll(Arrays.copyOfRange(residual, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));         
            calib.displayUpdate();
         }
      } //viz
      
   }


}


class KinematicCalibrationResidual implements FunctionNtoM
{

      //robot model and data
      private final SDFFullRobotModel fullRobotModel;
      private final ArrayList<Map<String, Double>> qdata;
      private final ArrayList<String> calJointNames;
      
      //local data buffer
      Map<String, Double> qoffset = new HashMap<>(), qbuffer = new HashMap<>();
      Vector3d constantOffset = new Vector3d();

      
      public KinematicCalibrationResidual(SDFFullRobotModel fullRobotModel, final ArrayList<String> calJointNames, ArrayList<Map<String, Double>> qdata)
      {
         this.fullRobotModel = fullRobotModel;
         this.calJointNames = calJointNames;         
         this.qdata = qdata;
      }

      public Map<String, Double> getqOffset()
      {
         return qoffset;
      }

      public Vector3d getConstantOffset()
      {
         return constantOffset;
      }
      
      public Map<String, Double> prmArrayToJointMap(double[] prm)
      {
         Map<String, Double> qret = new HashMap<>();
         assert(prm.length == calJointNames.size());
         for(int i=0;i<calJointNames.size();i++)
         {
            qret.put(calJointNames.get(i), prm[i]);
         }
         return qret; 
      }
      
      public double[] calcResiduals(double[] input)
      {
         double[] output = new double[getM()];
         process(input, output);
         return output;
      }

      
      @Override 
      public void process(double[] input, double[] output)
      {
         //convert input into map
         int inputCounter=0;
         for(int i=0;i<calJointNames.size();i++)
            qoffset.put(calJointNames.get(i), input[inputCounter++]);

         //remember to change getN()
//       constantOffset.x=input[inputCounter++];
         constantOffset.y=input[inputCounter++];
//       constantOffset.z=input[inputCounter++];
         
         //compute error            
         int outputCounter=0;
         for(int i=0;i<qdata.size();i++)
         {
            CalibUtil.addQ(qdata.get(i),qoffset, qbuffer);
            CalibUtil.setRobotModelFromData(fullRobotModel,qbuffer);
            
//            FramePoint 
//               leftEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM)  ,+0.01, 0.13,0),
//               rightEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM),+0.01,-0.13,0);
            FramePose leftEE  = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT,  LimbName.ARM),new Point3d(+0.01,+0.13,0), CalibUtil.quat0);
            FramePose rightEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM),new Point3d(+0.01,-0.13,0), CalibUtil.quat0);
            leftEE.addPositionInFrame(constantOffset);

            leftEE.changeFrame(ReferenceFrame.getWorldFrame());
            rightEE.changeFrame(ReferenceFrame.getWorldFrame());
            output[outputCounter++] = leftEE.getX()-rightEE.getX();
            output[outputCounter++] = leftEE.getY()-rightEE.getY();
            output[outputCounter++] = leftEE.getZ()-rightEE.getZ();

            if (AtlasKinematicCalibrator.RESIDUAL_DOF==6)
            {

               double scaleRadToCM = 0.01/(Math.PI/2); //30deg -> 1cm
                 Quat4d qErr = leftEE.getOrientationCopy().getQuaternion();
                 qErr.inverse();
                 qErr.mul(rightEE.getOrientationCopy().getQuaternion());
                 AxisAngle4d axErr = new AxisAngle4d();
                 axErr.set(qErr);
                 output[outputCounter++] = scaleRadToCM*axErr.getX()*axErr.getAngle();
                 output[outputCounter++] = scaleRadToCM*axErr.getY()*axErr.getAngle();
                 output[outputCounter++] = scaleRadToCM*axErr.getZ()*axErr.getAngle();
               }
         }
      }
      
      
      @Override
      public int getN()
      {
         //dim parameter
         return calJointNames.size() + 1; //+3 for constant offset
      }
      
      @Override
      public int getM()
      {
         //dim error
          return qdata.size()*AtlasKinematicCalibrator.RESIDUAL_DOF;
      }
}

