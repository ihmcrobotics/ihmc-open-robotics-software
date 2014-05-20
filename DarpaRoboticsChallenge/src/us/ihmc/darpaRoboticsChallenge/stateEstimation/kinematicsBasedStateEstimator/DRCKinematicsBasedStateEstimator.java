package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.sensors.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCStateEstimatorInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicPositionArtifact;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class DRCKinematicsBasedStateEstimator implements DRCStateEstimatorInterface, StateEstimator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final boolean INITIALIZE_HEIGHT_WITH_FOOT = false;
   
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final DoubleYoVariable yoTime = new DoubleYoVariable("t_stateEstimator", registry);
   
   private final JointStateUpdater jointStateUpdater;
   private final PelvisRotationalStateUpdater pelvisRotationalStateUpdater;
   private final PelvisLinearStateUpdater pelvisLinearStateUpdater;
 
   private final double estimatorDT;
   
   private boolean visualizeMeasurementFrames = false;
   private final ArrayList<DynamicGraphicReferenceFrame> dynamicGraphicMeasurementFrames = new ArrayList<>();

   private final SideDependentList<YoFramePoint> footRawCoPPositionsInWorld;
   private final YoFramePoint overallRawCoPPositionInWorld;
   private final FramePoint2d tempRawCoP2d;
   private final FramePoint tempRawCoP;
   private final Wrench tempWrench;
   private final SideDependentList<WrenchBasedFootSwitch> footSwitches;

   public DRCKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, StateEstimatorParameters stateEstimatorParameters,
         SensorOutputMapReadOnly sensorOutputMapReadOnly, double gravitationalAcceleration, SideDependentList<WrenchBasedFootSwitch> footSwitches,
         SideDependentList<ContactablePlaneBody> bipedFeet, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();

      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, registry);

      List<? extends IMUSensorReadOnly> imuProcessedOutputs = sensorOutputMapReadOnly.getIMUProcessedOutputs();

      pelvisRotationalStateUpdater = new PelvisRotationalStateUpdater(inverseDynamicsStructure, imuProcessedOutputs, registry);

      pelvisLinearStateUpdater = new PelvisLinearStateUpdater(inverseDynamicsStructure, imuProcessedOutputs, footSwitches, bipedFeet,
            gravitationalAcceleration, yoTime, stateEstimatorParameters, dynamicGraphicObjectsListRegistry, registry);
      

      if (dynamicGraphicObjectsListRegistry != null)
      {
         footRawCoPPositionsInWorld = new SideDependentList<>();
         tempRawCoP2d = new FramePoint2d();
         tempRawCoP = new FramePoint();
         this.footSwitches = footSwitches;
         tempWrench = new Wrench();
         
         for (RobotSide robotSide : RobotSide.values)
         {
            String side = robotSide.getCamelCaseNameForMiddleOfExpression();
            YoFramePoint rawCoPPositionInWorld = new YoFramePoint("raw" + side + "CoPPositionsInWorld", worldFrame, registry);
            footRawCoPPositionsInWorld.put(robotSide, rawCoPPositionInWorld);
            
            DynamicGraphicPositionArtifact copArtifact = rawCoPPositionInWorld.createDynamicGraphicPosition("Meas " + side + "CoP", 0.008, YoAppearance.DarkRed(), GraphicType.DIAMOND).createArtifact();
            dynamicGraphicObjectsListRegistry.registerArtifact("StateEstimator", copArtifact);
         }
         
         overallRawCoPPositionInWorld = new YoFramePoint("overallRawCoPPositionInWorld", worldFrame, registry);
         DynamicGraphicPositionArtifact overallRawCoPArtifact = overallRawCoPPositionInWorld.createDynamicGraphicPosition("Meas CoP", 0.015, YoAppearance.DarkRed(), GraphicType.DIAMOND).createArtifact();
         dynamicGraphicObjectsListRegistry.registerArtifact("StateEstimator", overallRawCoPArtifact);
      }
      else
      {
         footRawCoPPositionsInWorld = null;
         overallRawCoPPositionInWorld = null;
         tempRawCoP2d = null;
         tempRawCoP = null;
         this.footSwitches = null;
         tempWrench = null;
      }
      
      visualizeMeasurementFrames = visualizeMeasurementFrames && dynamicGraphicObjectsListRegistry != null;
      
      if (visualizeMeasurementFrames)
         setupDynamicGraphicObjects(dynamicGraphicObjectsListRegistry, imuProcessedOutputs);
   }

   private void setupDynamicGraphicObjects(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         List<? extends IMUSensorReadOnly> imuProcessedOutputs)
   {
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         DynamicGraphicReferenceFrame dynamicGraphicMeasurementFrame = new DynamicGraphicReferenceFrame(imuProcessedOutputs.get(i).getMeasurementFrame(), registry, 1.0);
         dynamicGraphicMeasurementFrames.add(dynamicGraphicMeasurementFrame);
      }
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("imuFrame", dynamicGraphicMeasurementFrames);
   }

   public StateEstimator getStateEstimator()
   {
      return this;
   }

   public void initialize()
   {
      jointStateUpdater.initialize();
      pelvisRotationalStateUpdater.initialize();
      pelvisLinearStateUpdater.initialize();
   }
   
   public void doControl()
   {
      yoTime.add(estimatorDT); //Hack to have a yoTime in the state estimator

      jointStateUpdater.updateJointState();
      pelvisRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();
      pelvisLinearStateUpdater.updateRootJointPositionAndLinearVelocity();

      updateVisualizers();
   }

   private void updateVisualizers()
   {
      if (footRawCoPPositionsInWorld != null)
      {
         overallRawCoPPositionInWorld.setToZero();
         double totalFootForce = 0.0;
         
         for (RobotSide robotSide : RobotSide.values)
         {
            footSwitches.get(robotSide).computeAndPackCoP(tempRawCoP2d);
            tempRawCoP.setIncludingFrame(tempRawCoP2d.getReferenceFrame(), tempRawCoP2d.getX(), tempRawCoP2d.getY(), 0.0);
            tempRawCoP.changeFrame(worldFrame);
            footRawCoPPositionsInWorld.get(robotSide).set(tempRawCoP);
            
            footSwitches.get(robotSide).computeAndPackFootWrench(tempWrench);
            double singleFootForce = tempWrench.getLinearPartZ();
            totalFootForce += singleFootForce;
            tempRawCoP.scale(singleFootForce);
            overallRawCoPPositionInWorld.add(tempRawCoP);
         }
         
         overallRawCoPPositionInWorld.scale(1.0 / totalFootForce);
      }
      
      if (visualizeMeasurementFrames)
      {
         for (int i = 0; i < dynamicGraphicMeasurementFrames.size(); i++)
            dynamicGraphicMeasurementFrames.get(i).update();
      }
   }

   public void startIMUDriftEstimation()
   {
      pelvisLinearStateUpdater.startIMUDriftEstimation();
   }

   public void startIMUDriftCompensation()
   {
      pelvisLinearStateUpdater.startIMUDriftCompensation();
   }
   
   public void initializeEstimatorToActual(Point3d initialCoMPosition, Quat4d initialEstimationLinkOrientation)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(initialCoMPosition);
      // Do nothing for the orientation since the IMU is trusted
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void getEstimatedOrientation(FrameOrientation estimatedOrientationToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedOrientation(estimatedOrientationToPack);
   }

   public void setEstimatedOrientation(FrameOrientation estimatedOrientation)
   {
      // Do nothing, IMU is trusted
   }

   public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedAngularVelocity(estimatedAngularVelocityToPack);
   }

   public void setEstimatedAngularVelocity(FrameVector estimatedAngularVelocity)
   {
      // Do nothing, IMU is trusted
   }

   public void getEstimatedCoMPosition(FramePoint estimatedCoMPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMPosition(estimatedCoMPositionToPack);
   }

   public void setEstimatedCoMPosition(FramePoint estimatedCoMPosition)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(estimatedCoMPosition);
   }

   public void getEstimatedCoMVelocity(FrameVector estimatedCoMVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMVelocity(estimatedCoMVelocityToPack);
   }

   public void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity)
   {
   }

   public void getEstimatedPelvisPosition(FramePoint estimatedPelvisPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisPosition(estimatedPelvisPositionToPack);
   }

   public void getEstimatedPelvisLinearVelocity(FrameVector estimatedPelvisLinearVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisLinearVelocity(estimatedPelvisLinearVelocityToPack);
   }

   public DenseMatrix64F getCovariance()
   {
      return null;
   }

   public DenseMatrix64F getState()
   {
      return null;
   }

   public void setState(DenseMatrix64F x, DenseMatrix64F covariance)
   {
   }

   public void initializeOrientationEstimateToMeasurement()
   {
      // Do nothing
   }
}
