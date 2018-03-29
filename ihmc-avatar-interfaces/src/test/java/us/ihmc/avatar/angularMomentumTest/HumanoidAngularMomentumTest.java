package us.ihmc.avatar.angularMomentumTest;

import java.util.ArrayList;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class HumanoidAngularMomentumTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCRobotModel robotModel;
   private FullHumanoidRobotModel fullRobotModel;
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private double defaultSwingDuration = 1;
   private double defaultTransferDuration = 0.1;
   private double defaultFinalTransferDuration = 0.1;

   public void walk() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      AngularMomentumSpy angularMomentumSpy = new AngularMomentumSpy(drcSimulationTestHelper);
      double stepLength = getStepLength();
      double stepWidth = getStepWidth();
      RobotSide side = RobotSide.LEFT;
      FootstepDataListMessage footMessage = new FootstepDataListMessage();
      int numberOfFootstepsToTake = getNumberOfStepsToTake();
      Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      Point3D footLocation = new Point3D();
      double x = 0.0;
      for (int i = 0; i < numberOfFootstepsToTake; i++)
      {
         x += stepLength;
         footLocation.set(x, side.negateIfRightSide(stepWidth) / 2, 0.0);
         addFootstep(footLocation, footOrientation, side, defaultSwingDuration, defaultTransferDuration, footMessage);
         side = side.getOppositeSide();
      }
      footLocation.set(x, side.negateIfRightSide(stepWidth / 2), 0.0);
      addFootstep(footLocation, footOrientation, side, defaultSwingDuration, defaultTransferDuration, footMessage);
      footMessage.setFinalTransferDuration(defaultFinalTransferDuration);
      angularMomentumSpy.setFootstepList(footMessage);
      double testTime = numberOfFootstepsToTake * 2.0 + 1.0;
      drcSimulationTestHelper.send(footMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(testTime);
      destroySimulationAndRecycleMemory();
   }

   private void setupTest()
   {
      robotModel = getRobotModel();
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();
      simulationTestingParameters.setKeepSCSUp(true);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(emptyEnvironment);
      drcSimulationTestHelper.createSimulation(className);
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      ThreadTools.sleep(1000);
   }

   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }
      simulationTestingParameters = null;
   }

   private void addFootstep(Point3D stepLocation, Quaternion orient, RobotSide robotSide, double swingTime, double transferTime,
                            FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setLocation(stepLocation);
      footstepData.setOrientation(orient);
      footstepData.setRobotSide(robotSide.toByte());
      footstepData.setSwingDuration(swingTime);
      footstepData.setTransferDuration(transferTime);
      message.footstepDataList.add().set(footstepData);
   }

   protected double getStepLength()
   {
      return 0.25;
   }

   protected double getStepWidth()
   {
      return 0.08;
   }

   protected int getNumberOfStepsToTake()
   {
      return 10;
   }

   private class AngularMomentumSpy extends SimpleRobotController
   {
      HumanoidFloatingRootJointRobot floatingRootJointModel;
      Vector3D comAngMom = new Vector3D();
      Vector3D comLinMom = new Vector3D();
      Point3D comPoint = new Point3D();
      YoFrameVector comAngularMomentum;
      YoFrameVector comEstimatedAngularMomentum;
      FloatingJoint rootJoint;
      Quaternion rootOrientation = new Quaternion();
      RigidBodyTransform rootJointTransform = new RigidBodyTransform();
      FootstepDataListMessage footstepListMessage;
      SimulationConstructionSet scs;
      double t, temp, temp2, phaseTime, footstepDuration;
      ArrayList<FootstepDataMessage> footstepList;
      Point3D fromPoint = new Point3D(), toPoint = new Point3D(), supportPoint = new Point3D(), comLoc = new Point3D(), comLocIn = new Point3D(),
            comLocFi = new Point3D(), swFootLoc = new Point3D(), tempPoint = new Point3D();
      Vector3D swFootVelo = new Vector3D(), swFootVeloIn = new Vector3D(), swFootVeloFi = new Vector3D(), comVelo = new Vector3D(), comVeloIn = new Vector3D(),
            comVeloFi = new Vector3D();
      Vector3D swVectorAngMom = new Vector3D();
      boolean transferPhase;
      YoPolynomial3D swTraj, comTraj;
      Point3D entryCMP = new Point3D(-0.01, 0.005, 0.0);
      Point3D exitCMP =  new Point3D(0.01, 0.0125, 0.0);
      
      Point3D entryFootCMP = new Point3D();
      Point3D exitFootCMP = new Point3D();      
      double swFootMass = 1.5;
      
      double footLift = 0.1;
      RobotSide robotSide = RobotSide.LEFT;
      
      public AngularMomentumSpy(DRCSimulationTestHelper simulationTestHelper)
      {
         YoVariableRegistry scsRegistry = drcSimulationTestHelper.getYoVariableRegistry();
         drcSimulationTestHelper.addRobotControllerOnControllerThread(this);
         floatingRootJointModel = drcSimulationTestHelper.getRobot();
         rootJoint = floatingRootJointModel.getRootJoint();
         comAngularMomentum = new YoFrameVector("CoMAngularMomentum", worldFrame, scsRegistry);
         comEstimatedAngularMomentum = new YoFrameVector("CoMEstimatedAngularMomentum", worldFrame, scsRegistry);
         scs = drcSimulationTestHelper.getSimulationConstructionSet();
         swTraj = new YoPolynomial3D("SwingFootTraj", 4, scsRegistry);
         comTraj = new YoPolynomial3D("CoMTraj", 4, scsRegistry);
      }

      @Override
      public void doControl()
      {
         floatingRootJointModel.computeCOMMomentum(comPoint, comLinMom, comAngMom);
         rootJoint.getRotationToWorld(rootOrientation);
         rootJointTransform.setRotationAndZeroTranslation(rootOrientation);
         rootJointTransform.inverseTransform(comAngMom);
         comAngularMomentum.set(comAngMom);
         t = scs.getTime();
         temp = 0;
         int index;

         for (index = 0; index < footstepList.size(); index++)
         {
            temp += footstepList.get(index).getTransferDuration();
            if (temp > t)
            {
               phaseTime = footstepList.get(index).getTransferDuration();
               transferPhase = true;
               temp2 = temp + footstepList.get(index).getSwingDuration();
               footstepDuration = footstepList.get(index).getTransferDuration() + footstepList.get(index).getSwingDuration();
               break;
            }
            temp += footstepList.get(index).getSwingDuration();
            if (temp > t)
            {
               phaseTime = footstepList.get(index).getSwingDuration();
               transferPhase = false;
               temp2 = temp;
               footstepDuration = footstepList.get(index).getTransferDuration() + footstepList.get(index).getSwingDuration();
               break;
            }
         }         
         
         if (index > 1 && index < footstepList.size())
         {
            fromPoint = footstepList.get(index - 2).getLocation();
            toPoint = footstepList.get(index).getLocation();
            supportPoint = footstepList.get(index - 1).getLocation();
            robotSide = RobotSide.fromByte(footstepList.get(index).getRobotSide());
            entryFootCMP.set(entryCMP.getX(), robotSide.negateIfLeftSide(entryCMP.getY()), entryCMP.getZ());
            exitFootCMP.set(exitCMP.getX(), robotSide.negateIfLeftSide(exitCMP.getY()), exitCMP.getZ());
            
            double tphase = (t - temp + phaseTime);
            if (transferPhase)
            {
//               comLocIn.set(fromPoint);
//               comLocIn.add(exitFootCMP);
//               comLocFi.set(supportPoint);
//               comLocFi.add(entryFootCMP);
//               comVeloIn.set(swFootVeloFi);
//               comVeloIn.scale(-1.0);
//               comVeloIn.setToZero();
//               comVeloFi.setToZero();
//               comTraj.setCubic(0.0, phaseTime, comLocIn, comVeloIn, comLocFi, comVeloFi);
//               comTraj.compute(tphase);
//               comLoc.set(comTraj.getPosition());
//               comVelo.set(comTraj.getVelocity());
               swFootLoc.set(fromPoint);
               swFootVelo.setToZero();
            }
            else
            {
//               comLocIn.set(supportPoint);
//               comLocIn.add(entryFootCMP);
//               comLocFi.set(supportPoint);
//               comLocFi.add(exitFootCMP);
//               comVeloIn.setToZero();
//               comVeloFi.setToZero();
//               comTraj.setCubic(0.0, phaseTime, comLocIn, comVeloIn, comLocFi, comVeloFi);
//               comTraj.compute(tphase);
//               comLoc.set(comTraj.getPosition());
//               comVelo.set(comTraj.getVelocity());
               swFootVeloIn.set(0.0, 0.0, 0.0);
//               swFootVeloFi.set(fromPoint);
//               swFootVeloFi.sub(toPoint);
//               swFootVeloFi.scale(1.0/phaseTime);
               swFootVeloFi.setToZero();
               swTraj.setCubic(0, phaseTime, fromPoint, swFootVeloIn, toPoint, swFootVeloFi);
               swTraj.compute(tphase);
               swFootLoc.set(swTraj.getPosition());
               swFootLoc.add(0.0, 0.0, footLift * 4 * tphase * (1.0 - tphase/phaseTime)/phaseTime);
               swFootVelo.set(swTraj.getVelocity());
            }
            double tPh2 = (t - temp2 + footstepDuration)/footstepDuration;
            
            tempPoint.set(fromPoint);
            tempPoint.add(exitFootCMP);
            tempPoint.scale(Math.pow((1.0 - tPh2), 3));
            comLoc.set(tempPoint);
            
            tempPoint.set(supportPoint);
            tempPoint.add(entryFootCMP);
            tempPoint.scale(3*Math.pow((1.0 - tPh2), 2)*tPh2);
            comLoc.add(tempPoint);
            
            tempPoint.set(supportPoint);
            tempPoint.add(exitFootCMP);
            tempPoint.scale(3*Math.pow(tPh2, 2)*(1.0 - tPh2));
            comLoc.add(tempPoint);
            
            tempPoint.set(toPoint);
            tempPoint.add(entryFootCMP);
            tempPoint.scale(Math.pow(tPh2, 3));
            comLoc.add(tempPoint);
            
            tempPoint.set(supportPoint);
            tempPoint.add(entryFootCMP);
            tempPoint.sub(fromPoint);
            tempPoint.sub(exitFootCMP);
            tempPoint.scale(3*Math.pow((1.0 - tPh2), 2));
            comVelo.set(tempPoint);
            
            tempPoint.set(exitFootCMP);
            tempPoint.sub(entryFootCMP);
            tempPoint.scale(6*(1.0 - tPh2)*tPh2);
            comVelo.add(tempPoint);
            
            tempPoint.set(toPoint);
            tempPoint.add(entryFootCMP);
            tempPoint.sub(supportPoint);
            tempPoint.sub(exitFootCMP);
            tempPoint.scale(3*Math.pow(tPh2, 2));
            comVelo.add(tempPoint);
            
            comLoc.add(0.0, 0.0, 0.33);
            swVectorAngMom.set(swFootLoc);
            swVectorAngMom.sub(comLoc);
            swFootVelo.sub(comVelo);
            swVectorAngMom.cross(swVectorAngMom, swFootVelo);
            swVectorAngMom.scale(swFootMass);
            comEstimatedAngularMomentum.set(swVectorAngMom);
         }
      }

      public void setFootstepList(FootstepDataListMessage footMessage)
      {
         this.footstepListMessage = footMessage;
         this.footstepList = new ArrayList<>();
         for (int i = 0; i < this.footstepListMessage.getFootstepDataList().size(); i++)
            footstepList.add(this.footstepListMessage.getFootstepDataList().get(i));
      }
   }
}