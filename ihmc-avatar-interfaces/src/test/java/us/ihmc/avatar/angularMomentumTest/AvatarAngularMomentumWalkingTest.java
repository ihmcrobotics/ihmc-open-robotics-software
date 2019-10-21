package us.ihmc.avatar.angularMomentumTest;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Random;

import org.apache.commons.io.IOUtils;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.EuclideanTrajectoryMessage;
import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.MomentumTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class AvatarAngularMomentumWalkingTest implements MultiRobotTestInterface
{
   private static final String fileName = "angularMomentumData.txt";
   private static final double angularMomentumRecordDT = 0.05; // sim time between data points
   private static final boolean keepSCSUp = false;

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;
   private final Random random = new Random(1738);

   // default step parameters for flat ground walking

   protected abstract double getStepLength();

   protected abstract double getStepWidth();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters.setKeepSCSUp(keepSCSUp && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void setupTest()
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      PrintTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(className);
      robotModel = getRobotModel();

      ThreadTools.sleep(1000);
   }

   @Test
   public void testForwardWalkWithAngularMomentumReference() throws SimulationExceededMaximumTimeException
   {
      // only set to true when saving new angular momentum data. output file usually needs to be manually moved to resources folder
      boolean exportAchievedAngularMomentum = false;

      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      AngularMomentumRecorder recordingScript = null;
      if(exportAchievedAngularMomentum)
      {
         recordingScript = new AngularMomentumRecorder(drcSimulationTestHelper);
         recordingScript.markStart();
      }

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps, false));

      MomentumTrajectoryMessage momentumTrajectoryMessage = loadFileToMessage();
      drcSimulationTestHelper.publishToController(momentumTrajectoryMessage);

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      if(exportAchievedAngularMomentum)
         recordingScript.exportToFile();
   }

   @Test
   public void testForwardWalkWithCorruptedMomentum() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps, true));

      MomentumTrajectoryMessage momentumTrajectoryMessage = loadFileToMessage();
      addTimeCorruption(momentumTrajectoryMessage);
      drcSimulationTestHelper.publishToController(momentumTrajectoryMessage);

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @Test
   public void testWalkingWithDelayedMomentum() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps, true));

      MomentumTrajectoryMessage momentumTrajectoryMessage = loadFileToMessage();
      double timeDelay = 1.5 * initialTransfer;
      addTimeDelay(momentumTrajectoryMessage, timeDelay);
      drcSimulationTestHelper.publishToController(momentumTrajectoryMessage);

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @Test
   public void testForwardWalkZeroMomentumFirstStep() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      MomentumTrajectoryMessage momentumTrajectoryMessage = loadFileToMessage();
      setTimeRangeToZero(momentumTrajectoryMessage, 0.0, initialTransfer + swing);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps, true));
      drcSimulationTestHelper.publishToController(momentumTrajectoryMessage);

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @Test
   public void testWalkingWithRandomSinusoidalMomentum() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps, true));
      drcSimulationTestHelper.publishToController(createRandomSinusoidalMomentumTrajectoryMessage(initialTransfer, transfer, swing, numberOfSteps + 1, random));

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   private FootstepDataListMessage createFootstepMessage(int numberOfSteps, boolean addTimeCorruption)
   {
      RobotSide side = RobotSide.LEFT;
      double stepLength = getStepLength();
      double stepWidth = getStepWidth();
      FootstepDataListMessage message = new FootstepDataListMessage();

      for (int currentStep = 0; currentStep < numberOfSteps; currentStep++)
      {
         Point3D footLocation = new Point3D(stepLength * (currentStep + 1), side.negateIfRightSide(stepWidth / 2), 0.0);
         Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrientation));
         side = side.getOppositeSide();
      }

      Point3D footLocation = new Point3D(stepLength * (numberOfSteps), side.negateIfRightSide(stepWidth / 2), 0.0);
      Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrientation));

      if(addTimeCorruption)
         addCorruptionToFootstepTimings(message);

      return message;
   }

   private void addCorruptionToFootstepTimings(FootstepDataListMessage message)
   {
      for (int i = 0; i < message.getFootstepDataList().size(); i++)
      {
         double transferCorruption = RandomNumbers.nextDouble(random, 0.02);
         double swingCorruption = RandomNumbers.nextDouble(random, 0.02);

         double transferDuration = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
         double swingDuration = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();

         transferDuration += transferCorruption;
         swingDuration += swingCorruption;

         message.getFootstepDataList().get(i).setTransferDuration(transferDuration);
         message.getFootstepDataList().get(i).setSwingDuration(swingDuration);
      }
   }

   private void addTimeCorruption(MomentumTrajectoryMessage momentumTrajectoryMessage)
   {
      EuclideanTrajectoryMessage angularMomentumTrajectory = momentumTrajectoryMessage.getAngularMomentumTrajectory();
      Object<EuclideanTrajectoryPointMessage> taskspaceTrajectoryPoints = angularMomentumTrajectory.getTaskspaceTrajectoryPoints();

      for (int i = 0; i < taskspaceTrajectoryPoints.size(); i++)
      {
         EuclideanTrajectoryPointMessage trajectoryPoint = taskspaceTrajectoryPoints.get(i);
         double timeDeviation = EuclidCoreRandomTools.nextDouble(random, 0.25 * angularMomentumRecordDT);
         trajectoryPoint.setTime(trajectoryPoint.getTime() + timeDeviation);
      }
   }

   private void addTimeDelay(MomentumTrajectoryMessage momentumTrajectoryMessage, double timeDelay)
   {
      EuclideanTrajectoryMessage angularMomentumTrajectory = momentumTrajectoryMessage.getAngularMomentumTrajectory();
      Object<EuclideanTrajectoryPointMessage> taskspaceTrajectoryPoints = angularMomentumTrajectory.getTaskspaceTrajectoryPoints();

      for (int i = 0; i < taskspaceTrajectoryPoints.size(); i++)
      {
         EuclideanTrajectoryPointMessage trajectoryPoint = taskspaceTrajectoryPoints.get(i);
         trajectoryPoint.setTime(trajectoryPoint.getTime() + timeDelay);
      }
   }

   private void setTimeRangeToZero(MomentumTrajectoryMessage momentumTrajectoryMessage, double t0, double tf)
   {
      EuclideanTrajectoryMessage angularMomentumTrajectory = momentumTrajectoryMessage.getAngularMomentumTrajectory();
      Object<EuclideanTrajectoryPointMessage> taskspaceTrajectoryPoints = angularMomentumTrajectory.getTaskspaceTrajectoryPoints();

      for (int i = 0; i < taskspaceTrajectoryPoints.size(); i++)
      {
         EuclideanTrajectoryPointMessage trajectoryPoint = taskspaceTrajectoryPoints.get(i);
         if(trajectoryPoint.getTime() > tf)
         {
            break;
         }
         else if(trajectoryPoint.getTime() > t0)
         {
            trajectoryPoint.getPosition().setToZero();
            trajectoryPoint.getLinearVelocity().setToZero();
         }
      }
   }

   private MomentumTrajectoryMessage createRandomSinusoidalMomentumTrajectoryMessage(double initialTransferDuration, double transferDuration, double swingDuration,
                                                                           int numberOfSteps, Random random)
   {
      MomentumTrajectoryMessage momentumTrajectoryMessage = new MomentumTrajectoryMessage();
      EuclideanTrajectoryMessage angularMomentum = momentumTrajectoryMessage.getAngularMomentumTrajectory();

      double dt = 0.001;
      for (double time = 0; time <= initialTransferDuration; time += dt)
      {
         double timeCorruption = RandomNumbers.nextDouble(random, 0.5 * dt);
         EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
         point.setTime(time + timeCorruption);
         point.getPosition().setToZero();
         point.getLinearVelocity().setToZero();
      }

      RobotSide robotSide = RobotSide.LEFT;
      double timeCorruption = RandomNumbers.nextDouble(random, 0.01);
      double currentTime = initialTransferDuration + timeCorruption;
      double angularMomentumXMagnitude = -3.0;
      double angularMomentumYMagnitude = -10.0;
      double angularMomentumXFrequency = 2.0 * Math.PI / swingDuration;
      double angularMomentumYFrequency = 2.0 * Math.PI / (2.0 * swingDuration);
      for (double time = 0; time <= swingDuration; time += dt)
      {
         double dtCorruption = RandomNumbers.nextDouble(random, 0.5 * dt);

         EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
         point.setTime(currentTime + time + dtCorruption);

         double xMomentum = angularMomentumXMagnitude * Math.sin(angularMomentumXFrequency * time);
         double yMomentum = angularMomentumYMagnitude * Math.sin(angularMomentumYFrequency * time);
         yMomentum = robotSide.negateIfRightSide(yMomentum);

         point.getPosition().set(xMomentum, yMomentum, 0.0);
         point.getLinearVelocity().setToZero();
      }

      timeCorruption = RandomNumbers.nextDouble(random, 0.01);
      currentTime += swingDuration + timeCorruption;

      for (int stepNumber = 1; stepNumber < numberOfSteps; stepNumber++)
      {
         for (double time = 0; time < transferDuration; time += dt)
         {
            double dtCorruption = RandomNumbers.nextDouble(random, 0.5 * dt);

            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time + dtCorruption);
            point.getPosition().setToZero();
            point.getLinearVelocity().setToZero();
         }

         timeCorruption = RandomNumbers.nextDouble(random, 0.01);
         currentTime += transferDuration + timeCorruption;

         for (double time = 0; time < swingDuration; time += dt)
         {
            double dtCorruption = RandomNumbers.nextDouble(random, 0.5 * dt);

            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time + dtCorruption);

            double xMomentum = angularMomentumXMagnitude * Math.sin(angularMomentumXFrequency * time);
            double yMomentum = angularMomentumYMagnitude * Math.sin(angularMomentumYFrequency * time);
            yMomentum = robotSide.negateIfRightSide(yMomentum);

            point.getPosition().set(xMomentum, yMomentum, 0.0);
            point.getLinearVelocity().setToZero();
         }

         timeCorruption = RandomNumbers.nextDouble(random, 0.01);
         currentTime += swingDuration + timeCorruption;
      }

      return momentumTrajectoryMessage;
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   // Angular momentum recording/loading

   class AngularMomentumRecorder implements Script
   {
      private int recordCounter = 0;
      private final int recordFrequency;
      private double startTime = 0.0;

      private final TDoubleArrayList times = new TDoubleArrayList();
      private final TDoubleArrayList achievedAngularMomentumX = new TDoubleArrayList();
      private final TDoubleArrayList achievedAngularMomentumY = new TDoubleArrayList();
      private final TDoubleArrayList achievedAngularMomentumZ = new TDoubleArrayList();

      private final YoDouble angularMomentumX;
      private final YoDouble angularMomentumY;
      private final YoDouble angularMomentumZ;
      private final YoDouble time;

      AngularMomentumRecorder(DRCSimulationTestHelper helper)
      {
         this.recordFrequency = (int) Math.round(angularMomentumRecordDT / helper.getSimulationConstructionSet().getDT());

         this.angularMomentumX = (YoDouble) helper.getYoVariable("AngularMomentumX");
         this.angularMomentumY = (YoDouble) helper.getYoVariable("AngularMomentumY");
         this.angularMomentumZ = (YoDouble) helper.getYoVariable("AngularMomentumZ");
         this.time = (YoDouble) helper.getYoVariable("t");

         drcSimulationTestHelper.getSimulationConstructionSet().addScript(this);
         recordCounter = recordFrequency - 1;
      }

      @Override
      public void doScript(double t)
      {
         if(++recordCounter >= recordFrequency)
         {
            times.add(t - startTime);
            achievedAngularMomentumX.add(angularMomentumX.getDoubleValue());
            achievedAngularMomentumY.add(angularMomentumY.getDoubleValue());
            achievedAngularMomentumZ.add(angularMomentumZ.getDoubleValue());

            recordCounter = 0;
         }
      }

      public void markStart()
      {
         startTime = time.getDoubleValue();
      }

      private void exportToFile()
      {
         try
         {
            Path filePath = Paths.get(fileName);
            File file = filePath.toFile();
            BufferedWriter writer = new BufferedWriter(new FileWriter(file.getName()));

            for (int i = 0; i < times.size(); i++)
            {
               double time = times.get(i);
               double angularMomentumX = achievedAngularMomentumX.get(i);
               double angularMomentumY = achievedAngularMomentumY.get(i);
               double angularMomentumZ = achievedAngularMomentumZ.get(i);
               writeLine(writer, time, angularMomentumX, angularMomentumY, angularMomentumZ);
            }

            writer.flush();
            writer.close();
            PrintTools.info("Exported angular momentum data to file: " + fileName);
         }
         catch (IOException e)
         {
            PrintTools.error("Unable to export angular momentum data. Tried to write to file: " + fileName);
            e.printStackTrace();
         }
      }

      private void writeLine(BufferedWriter writer, double time, double angularMomentumX, double angularMomentumY, double angularMomentumZ) throws IOException
      {
         writer.append(Double.toString(time));
         writer.append(',');
         writer.append(Double.toString(angularMomentumX));
         writer.append(',');
         writer.append(Double.toString(angularMomentumY));
         writer.append(',');
         writer.append(Double.toString(angularMomentumZ));
         writer.newLine();
      }
   }

   private static MomentumTrajectoryMessage loadFileToMessage()
   {
      try
      {
         ClassLoader classLoader = Thread.currentThread().getContextClassLoader();
         List<String> trajectoryDataList = (List<String>) IOUtils.readLines(classLoader.getResourceAsStream(fileName), "UTF-8");
         
         MomentumTrajectoryMessage trajectoryMessage = new MomentumTrajectoryMessage();
         us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.EuclideanTrajectoryPointMessage> trajectoryPoints = trajectoryMessage.getAngularMomentumTrajectory().getTaskspaceTrajectoryPoints();

         for (int i = 0; i < trajectoryDataList.size(); i++)
         {
            EuclideanTrajectoryPointMessage trajectoryPoint = trajectoryPoints.add();

            String[] trajectoryPointData = trajectoryDataList.get(i).split(",");
            trajectoryPoint.setTime(Double.parseDouble(trajectoryPointData[0]));
            trajectoryPoint.getPosition().setX(Double.parseDouble(trajectoryPointData[1]));
            trajectoryPoint.getPosition().setY(Double.parseDouble(trajectoryPointData[2]));
            trajectoryPoint.getPosition().setZ(Double.parseDouble(trajectoryPointData[3]));
         }

         // numerically approximate angular momentum rate for now. we should record this as a yo-variable in the future to improve accuracy
         for (int i = 1; i < trajectoryPoints.size() - 1; i++)
         {
            EuclideanTrajectoryPointMessage p_prev = trajectoryPoints.get(i - 1);
            EuclideanTrajectoryPointMessage p_curr = trajectoryPoints.get(i);
            EuclideanTrajectoryPointMessage p_next = trajectoryPoints.get(i + 1);

            double dt = p_next.getTime() - p_prev.getTime();
            p_curr.getLinearVelocity().set(p_next.getPosition());
            p_curr.getLinearVelocity().sub(p_prev.getPosition());
            p_curr.getLinearVelocity().scale(1 / dt);
         }

         return trajectoryMessage;
      }
      catch(IOException e)
      {
         throw new RuntimeException("Unable to import angular momentum data. Tried to load file: " + fileName);
      }
   }
}
