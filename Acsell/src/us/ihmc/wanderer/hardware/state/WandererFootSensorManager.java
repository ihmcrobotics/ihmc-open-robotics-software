package us.ihmc.wanderer.hardware.state;

import java.util.ArrayList;
import java.util.EnumMap;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.acsell.hardware.state.AcsellActuatorState;
import us.ihmc.acsell.hardware.state.AcsellJointState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wanderer.hardware.WandererActuator;
import us.ihmc.wanderer.hardware.WandererJoint;
import us.ihmc.wanderer.parameters.WandererPhysicalProperties;


public class WandererFootSensorManager
{
   private static final WandererActuator[] leftFootSensorActuators = {WandererActuator.LEFT_KNEE,WandererActuator.LEFT_ANKLE_RIGHT,WandererActuator.LEFT_ANKLE_LEFT};
   private static final WandererActuator[] rightFootSensorActuators = {WandererActuator.RIGHT_KNEE,WandererActuator.RIGHT_ANKLE_RIGHT,WandererActuator.RIGHT_ANKLE_LEFT};
   private static final int PRESSURE_SENSORS_PER_ROW = 4;
   private static final int PRESSURE_SENSORS_PER_FOOT = leftFootSensorActuators.length*PRESSURE_SENSORS_PER_ROW;
   private static final double[] sensorYPositions = new double[]
         { -2.125 * 0.0254,  -0.750 * 0.0254, 0.750 * 0.0254, 2.125 * 0.0254};
   private static final double[] sensorXPositions = new double[]
         { 0.700 * 0.0254 - WandererPhysicalProperties.footLength/2.0,
           9.300 * 0.0254 - WandererPhysicalProperties.footLength/2.0,
           7.450 * 0.0254 - WandererPhysicalProperties.footLength/2.0,
           0.000 * 0.0254 - WandererPhysicalProperties.footLength/2.0
         };
   private static final double[][] leftSensorsToUse = {{1,1,1,1},{0,0,0,0},{0,0,0,0}};
   private static final double[][] rightSensorsToUse = {{1,0,1,1},{0,0,0,0},{0,0,0,0}};
   
   private final EnumMap<WandererJoint, AcsellJointState> jointStates;
   private final EnumMap<WandererActuator, AcsellActuatorState> actuatorStates;
   private final SideDependentList<DenseMatrix64F> footWrenches;
      
   private final YoVariableRegistry registry = new YoVariableRegistry("WandererFootManager");
   private final ArrayList<DoubleYoVariable> leftPressureSensorValues = new ArrayList<DoubleYoVariable>();
   private final ArrayList<DoubleYoVariable>  rightPressureSensorValues = new ArrayList<DoubleYoVariable>();
   private final SideDependentList<DoubleYoVariable> feetForceZ = new SideDependentList<DoubleYoVariable>();
   private final SideDependentList<DoubleYoVariable> feetTauX = new SideDependentList<DoubleYoVariable>();
   private final SideDependentList<DoubleYoVariable> feetTauY = new SideDependentList<DoubleYoVariable>();
   private final SideDependentList<DoubleYoVariable> feetCoPX = new SideDependentList<DoubleYoVariable>();
   private final SideDependentList<DoubleYoVariable> feetCoPY = new SideDependentList<DoubleYoVariable>();
   private final SideDependentList<ArrayList<DoubleYoVariable>> pressureSensorValues = new SideDependentList<ArrayList<DoubleYoVariable>>();
   
   public WandererFootSensorManager(SideDependentList<DenseMatrix64F> footWrenches, EnumMap<WandererJoint, AcsellJointState> jointStates, EnumMap<WandererActuator, AcsellActuatorState> actuatorStates, YoVariableRegistry parentRegistry)
   {
      this.jointStates = jointStates;
      this.actuatorStates = actuatorStates;
      this.footWrenches = footWrenches;
      for(int i=0;i<PRESSURE_SENSORS_PER_FOOT; i++)
      {
         leftPressureSensorValues.add(new DoubleYoVariable("leftFootSensor" + i, registry));
         rightPressureSensorValues.add(new DoubleYoVariable("rightFootSensor" + i, registry));
      }
      for(RobotSide side : RobotSide.values)
      {
         feetForceZ.put(side,new DoubleYoVariable(side.getLowerCaseName()+"WandererFootForceZ", registry));
         feetTauX.put(side,new DoubleYoVariable(side.getLowerCaseName()+"WandererFootTauX", registry));
         feetTauY.put(side,new DoubleYoVariable(side.getLowerCaseName()+"WandererFootTauY", registry));
         feetCoPX.put(side,new DoubleYoVariable(side.getLowerCaseName()+"WandererFootCoPX", registry));
         feetCoPY.put(side,new DoubleYoVariable(side.getLowerCaseName()+"WandererFootCoPY", registry));
      }
      pressureSensorValues.put(RobotSide.LEFT,leftPressureSensorValues);
      pressureSensorValues.put(RobotSide.RIGHT,rightPressureSensorValues);      
      
      parentRegistry.addChild(registry);
   }
   

   public void update()
   {
      collectPressureSensors();
      
      for(RobotSide side : RobotSide.values)
      {
         double footForceZ = 0.0;
         double footTauX = 0.0;
         double footTauY = 0.0;
      
         for(int sensor=0; sensor<PRESSURE_SENSORS_PER_FOOT; sensor++)
         {
            footForceZ += pressureSensorValues.get(side).get(sensor).getDoubleValue();
            footTauX += pressureSensorValues.get(side).get(sensor).getDoubleValue()*sensorYPositions[sensor % PRESSURE_SENSORS_PER_ROW];
            footTauY += pressureSensorValues.get(side).get(sensor).getDoubleValue()*sensorXPositions[sensor / PRESSURE_SENSORS_PER_ROW];
         }   
         
         feetForceZ.get(side).set(footForceZ);
         feetTauX.get(side).set(footTauX);
         feetTauY.get(side).set(footTauY);
         if(footForceZ>25)
         {
            feetCoPX.get(side).set(footTauY/footForceZ);
            feetCoPY.get(side).set(footTauX/footForceZ);
         }         
      
         footWrenches.get(side).set(5, footForceZ);
      }
   }
   
   private void collectPressureSensors()
   {
      int a_id = 0;
      int sensorIndex = 0;
      for(WandererActuator actuator : leftFootSensorActuators)
      {         
         for(int id = 0; id < PRESSURE_SENSORS_PER_ROW; id++)
         {
            sensorIndex = a_id*PRESSURE_SENSORS_PER_ROW + id;
            leftPressureSensorValues.get(sensorIndex).set(actuatorStates.get(actuator).getPressureSensor(id).getValue()*leftSensorsToUse[a_id][id]);
         }
         a_id++;
      }
      
      a_id = 0;
      for(WandererActuator actuator : rightFootSensorActuators)
      {         
         for(int id = 0; id < PRESSURE_SENSORS_PER_ROW; id++)
         {
            sensorIndex = a_id*PRESSURE_SENSORS_PER_ROW + id;
            rightPressureSensorValues.get(sensorIndex).set(actuatorStates.get(actuator).getPressureSensor(id).getValue()*rightSensorsToUse[a_id][id]);
         }
         a_id++;
      }
   }
   
}
