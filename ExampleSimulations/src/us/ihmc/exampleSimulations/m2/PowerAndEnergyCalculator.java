package us.ihmc.exampleSimulations.m2;

import javax.vecmath.Point3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PowerAndEnergyCalculator implements RobotController
{

   private final double MOTOR_TORQUE_SENSITIVITY = 0.179;    // 0.13; //[Nm/A]
   private final double GEAR_RATIO = 50.0;    // [Nm/A]
   private final double MOTOR_RESISTANCE = 0.611;    // 100219 pdn set from RB- 02111 [ohms]

   private final YoVariableRegistry registry = new YoVariableRegistry("PowerCalculator");

   private final M2Robot m2Robot;

   private final DoubleYoVariable[] jointTorques;
   private final DoubleYoVariable[] jointVelocities;
   private final DoubleYoVariable[] currents;
   private final DoubleYoVariable[] gearboxEfficiencies;
   private final DoubleYoVariable[] jointPowers;
   private final DoubleYoVariable[] jointMechanicalPowers;
   private final AlphaFilteredYoVariable[] maximumJointSpeeds;
   private final AlphaFilteredYoVariable[] maximumJointTorques;


   private final AlphaFilteredYoVariable[] unsignedJointSpeeds;
   private final AlphaFilteredYoVariable[] unsignedJointTorques;


// private final YoVariable[] averageJointSpeeds;
// private final YoVariable[] averageJointTorques;
   private final DoubleYoVariable[] peakMechanicalPowers;
   private final DoubleYoVariable[] averageMechanicalJointPowers;
   private final DoubleYoVariable[] totalMechanicalJointEnergy;

   private final DoubleYoVariable totalMass;

   private final DoubleYoVariable instantaneousMechanicalPower;
   private final DoubleYoVariable averageMechanicalPower;
   private final DoubleYoVariable totalMechanicalEnergy;
   private final DoubleYoVariable mechanicalCostOfTransport;
   
   private final DoubleYoVariable totalCurrentMag;
   private final DoubleYoVariable instantaneousElectricalPower;
   private final DoubleYoVariable averageElecticalPower;
   private final DoubleYoVariable totalElectricalEnergy;
   private final DoubleYoVariable electricalCostOfTransport;
   
   private final DoubleYoVariable previousTime;
   private final DoubleYoVariable runningTime;
   private final DoubleYoVariable totalDistance;
   private final DoubleYoVariable averageWalkSpeed;
   private final DoubleYoVariable xPosition;
   private final DoubleYoVariable yPosition;
   private final DoubleYoVariable xPositionPrevious;
   private final DoubleYoVariable yPositionPrevious;
   private boolean initializedPosition = false;



   private double startTime;
   private boolean setStartTime = false;

   private final M2HarmonicDriveEfficiencyCalculator harmonicDriveEfficiencyCalculator;

   private String name;

   public PowerAndEnergyCalculator(M2Robot m2Robot, String name)
   {
      this.name = name;
      this.m2Robot = m2Robot;


      xPosition = (DoubleYoVariable)m2Robot.getVariable("q_x");
      yPosition = (DoubleYoVariable)m2Robot.getVariable("q_y");

      harmonicDriveEfficiencyCalculator = new M2HarmonicDriveEfficiencyCalculator();

      String[] prefixes = new String[] {"left_", "right_"};
      String[] listOfJointNames = new String[]
      {
         "hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_roll", "ankle_pitch"
      };

      int numberOfJoints = listOfJointNames.length * prefixes.length;
      jointTorques = new DoubleYoVariable[numberOfJoints];
      jointVelocities = new DoubleYoVariable[numberOfJoints];
      currents = new DoubleYoVariable[numberOfJoints];
      gearboxEfficiencies = new DoubleYoVariable[numberOfJoints];
      jointPowers = new DoubleYoVariable[numberOfJoints];
      jointMechanicalPowers = new DoubleYoVariable[numberOfJoints];
      maximumJointSpeeds = new AlphaFilteredYoVariable[numberOfJoints];
      maximumJointTorques = new AlphaFilteredYoVariable[numberOfJoints];


      unsignedJointSpeeds = new AlphaFilteredYoVariable[numberOfJoints];
      unsignedJointTorques = new AlphaFilteredYoVariable[numberOfJoints];


//    averageJointSpeeds = new YoVariable[numberOfJoints];
//    averageJointTorques = new YoVariable[numberOfJoints];
      peakMechanicalPowers = new DoubleYoVariable[numberOfJoints];
      averageMechanicalJointPowers = new DoubleYoVariable[numberOfJoints];
      totalMechanicalJointEnergy = new DoubleYoVariable[numberOfJoints];

      totalMass = new DoubleYoVariable("totalMass", "Total mass of the robot [kg]", registry);
 
      instantaneousMechanicalPower = new DoubleYoVariable("instantaneousMechanicalPower", "Instaneous power [W]", registry);
      averageMechanicalPower = new DoubleYoVariable("averageMechanicalPower", "Time average power [W]", registry);
      totalMechanicalEnergy = new DoubleYoVariable("totalMechanicalEnergy", "Total energy used [J]", registry);
      mechanicalCostOfTransport = new DoubleYoVariable("mechanicalCostOfTransport", "Dimensionless cost of transport not considering electrical losses", registry);
      
      totalCurrentMag = new DoubleYoVariable("totalCurrentMag", "Magnitude of total instantaneous current to all motors [A]", registry);
      instantaneousElectricalPower = new DoubleYoVariable("instantaneousElectricalPower", "Instaneous power [W]", registry);
      averageElecticalPower = new DoubleYoVariable("averageElecticalPower", "Time average power [W]", registry);
      totalElectricalEnergy = new DoubleYoVariable("totalElectricalEnergy", "Total energy used [J]", registry);
      electricalCostOfTransport = new DoubleYoVariable("electricalCostOfTransport", "Dimensionless cost of transport including electrical losses", registry);
         
      previousTime = new DoubleYoVariable("previousTime", registry);
      runningTime = new DoubleYoVariable("runningTime", registry);

      totalDistance = new DoubleYoVariable("totalDistance", registry);
      averageWalkSpeed = new DoubleYoVariable("averageWalkSpeed", registry);

      xPositionPrevious = new DoubleYoVariable("xPositionPrevious", registry);
      yPositionPrevious = new DoubleYoVariable("yPositionPrevious", registry);


      previousTime.set(m2Robot.getTime());


      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(20.0, M2Simulation.DT);

      int counter = 0;
      for (int i = 0; i < prefixes.length; i++)
      {
         for (int j = 0; j < listOfJointNames.length; j++)
         {
            String prefixedJointName = prefixes[i] + listOfJointNames[j];
            jointTorques[counter] = (DoubleYoVariable)m2Robot.getVariable("tau_" + prefixedJointName);
            jointVelocities[counter] = (DoubleYoVariable)m2Robot.getVariable("qd_" + prefixedJointName);
            currents[counter] = new DoubleYoVariable("i_" + prefixedJointName, "Instantaneous current motors [A]", registry);
            gearboxEfficiencies[counter] = new DoubleYoVariable(prefixedJointName + "_gear_eff", "Gearbox efficiency", registry);
            jointPowers[counter] = new DoubleYoVariable(prefixedJointName + "_power", "Joint power [W]", registry);

            jointMechanicalPowers[counter] = new DoubleYoVariable(prefixedJointName + "_jointMechPower", "Instantaneous joint mechanical power", registry);
            peakMechanicalPowers[counter] = new DoubleYoVariable(prefixedJointName + "_peakMechanicalPower", registry);
            averageMechanicalJointPowers[counter] = new DoubleYoVariable(prefixedJointName + "_averageMechanicalJointPowers", "", registry);
            totalMechanicalJointEnergy[counter] = new DoubleYoVariable(prefixedJointName + "_totalMechanicalJointEnergy", "", registry);

            maximumJointSpeeds[counter] = new AlphaFilteredYoVariable(prefixedJointName + "_maximumJointSpeed", registry, alpha);
            maximumJointTorques[counter] = new AlphaFilteredYoVariable(prefixedJointName + "_maximumJointTorque", registry, alpha);

            unsignedJointSpeeds[counter] = new AlphaFilteredYoVariable(prefixedJointName + "_unsignedJointSpeed", registry, alpha);
            unsignedJointTorques[counter] = new AlphaFilteredYoVariable(prefixedJointName + "_unsignedJointTorque", registry, alpha);



//          averageJointSpeeds[counter] = new YoVariable(name + "_averageJointSpeed", "", registry);
//          averageJointTorques[counter] = new YoVariable(name + "_averageJointTorque", "", registry);

            counter++;
         }
      }
   }


   public void doControl()
   {
      double currentTime = m2Robot.getTime();
      double deltaT = currentTime - previousTime.getDoubleValue();
      if (!setStartTime)
      {
         startTime = currentTime;
         setStartTime = true;
      }

      runningTime.set(currentTime - startTime);

      // Need to calculate motor current for all joints
      Point3d comPoint = new Point3d();
      totalMass.set(m2Robot.computeCenterOfMass(comPoint));
      
      calculateEfficiencies();
      calculateCurrents();
      calculateJointPowers(deltaT);
      calculateSpeedsAndTorques(deltaT);
      calculateMechanicalPowers(deltaT);
      calculateWalkingDistanceAndSpeed();


      if (runningTime.getDoubleValue() > 0.0)
      {
         averageMechanicalPower.set(totalMechanicalEnergy.getDoubleValue() / runningTime.getDoubleValue());
         averageElecticalPower.set(totalElectricalEnergy.getDoubleValue() / runningTime.getDoubleValue());
      }
      
      previousTime.set(currentTime);
   }

   private void calculateCurrents()
   {
      totalCurrentMag.set(0.0);

      for (int i = 0; i < currents.length; i++)
      {
         currents[i].set(((jointTorques[i].getDoubleValue() / GEAR_RATIO) / MOTOR_TORQUE_SENSITIVITY) / gearboxEfficiencies[i].getDoubleValue());
         totalCurrentMag.set(totalCurrentMag.getDoubleValue() + Math.abs(currents[i].getDoubleValue()));
      }
   }

   private void calculateWalkingDistanceAndSpeed()
   {
//    totalDistance = new YoVariable("totalDistance", registry);
//    averageWalkSpeed = new YoVariable("averageWalkSpeed", registry);

      if (!initializedPosition)
      {
         xPositionPrevious.set(xPosition.getDoubleValue());
         yPositionPrevious.set(yPosition.getDoubleValue());

         initializedPosition = true;
      }

      double xDisplacement = xPosition.getDoubleValue() - xPositionPrevious.getDoubleValue();
      double yDisplacement = yPosition.getDoubleValue() - yPositionPrevious.getDoubleValue();




      totalDistance.set(totalDistance.getDoubleValue() + Math.sqrt(xDisplacement * xDisplacement - yDisplacement * yDisplacement));

      if (runningTime.getDoubleValue() > 0.0)
         averageWalkSpeed.set(totalDistance.getDoubleValue() / runningTime.getDoubleValue());

      xPositionPrevious.set(xPosition.getDoubleValue());
      yPositionPrevious.set(yPosition.getDoubleValue());
   }

   private void calculateEfficiencies()
   {
      for (int i = 0; i < gearboxEfficiencies.length; i++)
      {
         // Set to 1.0 for now
         gearboxEfficiencies[i].set(harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(jointVelocities[i].getDoubleValue() * GEAR_RATIO, jointTorques[i].getDoubleValue()));
      }
   }

   private void calculateJointPowers(double deltaT)
   {
      instantaneousElectricalPower.set(0.0);

      for (int i = 0; i < jointPowers.length; i++)
      {
         // voltage across resistor:
         double voltatgeFromResistance = Math.abs(currents[i].getDoubleValue()) * MOTOR_RESISTANCE;
         double backEMFvoltage = Math.abs(jointVelocities[i].getDoubleValue()) * GEAR_RATIO * MOTOR_TORQUE_SENSITIVITY;

         jointPowers[i].set(Math.abs(currents[i].getDoubleValue()) * (voltatgeFromResistance + backEMFvoltage));    //
         instantaneousElectricalPower.set(instantaneousElectricalPower.getDoubleValue() + jointPowers[i].getDoubleValue());
      }

      totalElectricalEnergy.set(totalElectricalEnergy.getDoubleValue() + instantaneousElectricalPower.getDoubleValue() * deltaT);
      electricalCostOfTransport.set(totalElectricalEnergy.getDoubleValue()/(totalMass.getDoubleValue() * 9.81 * totalDistance.getDoubleValue()));
   }

   private void calculateMechanicalPowers(double deltaT)
   {
      instantaneousMechanicalPower.set(0.0);

      for (int i = 0; i < jointPowers.length; i++)
      {
         jointMechanicalPowers[i].set(Math.abs(unsignedJointTorques[i].getDoubleValue() * unsignedJointSpeeds[i].getDoubleValue()));
         peakMechanicalPowers[i].set(Math.max(peakMechanicalPowers[i].getDoubleValue(), jointMechanicalPowers[i].getDoubleValue()));
         totalMechanicalJointEnergy[i].set(totalMechanicalJointEnergy[i].getDoubleValue() + jointMechanicalPowers[i].getDoubleValue() * deltaT);

         if (runningTime.getDoubleValue() > 0.0)
            averageMechanicalJointPowers[i].set(totalMechanicalJointEnergy[i].getDoubleValue() / runningTime.getDoubleValue());
         
         instantaneousMechanicalPower.set(instantaneousMechanicalPower.getDoubleValue() + jointMechanicalPowers[i].getDoubleValue());
      }
      
      
      totalMechanicalEnergy.set(totalMechanicalEnergy.getDoubleValue() + instantaneousMechanicalPower.getDoubleValue() * deltaT);
      mechanicalCostOfTransport.set(totalMechanicalEnergy.getDoubleValue()/(totalMass.getDoubleValue() * 9.81 * totalDistance.getDoubleValue()));

   }

   private void calculateSpeedsAndTorques(double deltaT)
   {
//    maximumJointSpeeds
//    maximumJointTorques
//    averageJointSpeeds
//    averageJointTorques
      for (int i = 0; i < jointPowers.length; i++)
      {
         unsignedJointSpeeds[i].update(Math.abs(jointVelocities[i].getDoubleValue()));
         unsignedJointTorques[i].update(Math.abs(jointTorques[i].getDoubleValue()));

         maximumJointSpeeds[i].update(Math.max(Math.abs(unsignedJointSpeeds[i].getDoubleValue()), maximumJointSpeeds[i].getDoubleValue()));
         maximumJointTorques[i].update(Math.max(Math.abs(unsignedJointTorques[i].getDoubleValue()), maximumJointTorques[i].getDoubleValue()));
      }
   }



   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void setUpGUI(SimulationConstructionSet scs)
   {
//    scs.setupVarGroup("Power", null, new String[] { "q_.*" });

      scs.setupGraphGroup("power", new String[][][]
      {
         {
            {"left_state", "right_state"}, {""}
         },
         {
            {"averageElecticalPower", "instantaneousElectricalPower"}, {"auto"}
         },
         {
            {"totalCurrentMag"}, {"auto"}
         },
         {
            {"totalElectricalEnergy"}, {"auto"}
         },
         {
            {"t"}, {"auto"}
         },
         {
            {"qd_x", "qd_y"}, {"auto"}
         },
         {
            {"q_right_hip_pitch"}, {"auto"}
         },
         {
            {"qd_right_hip_pitch"}, {"auto"}
         },
         {
            {"totalDistance"}, {"auto"}
         },
         {
            {"averageWalkSpeed"}, {"auto"}
         },
      }, 2);



//    String[] prefixes = new String[] {"left_", "right_"};
//        String[] listOfJointNames = new String[]
//        {
//           "hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_roll", "ankle_pitch"
//        };


      scs.setupGraphGroup("speed", new String[][][]
      {
         {
            {"left_state", "right_state"}, {""}
         },
         {
            {"", ""}, {"auto"}
         },
         {
            {"left_hip_yaw_maximumJointSpeed", "right_hip_yaw_maximumJointSpeed"}, {"auto"}
         },
         {
            {"left_hip_roll_maximumJointSpeed", "right_hip_roll_maximumJointSpeed"}, {"auto"}
         },
         {
            {"left_hip_pitch_maximumJointSpeed", "right_hip_pitch_maximumJointSpeed"}, {"auto"}
         },
         {
            {"left_knee_maximumJointSpeed", "right_knee_maximumJointSpeed"}, {"auto"}
         },
         {
            {"left_ankle_pitch_maximumJointSpeed", "right_ankle_pitch_maximumJointSpeed"}, {"auto"}
         },
         {
            {"left_ankle_roll_maximumJointSpeed", "right_ankle_roll_maximumJointSpeed"}, {"auto"}
         },
      }, 2);



      scs.setupGraphGroup("torque", new String[][][]
      {
         {
            {"left_state", "right_state"}, {""}
         },
         {
            {"", ""}, {"auto"}
         },
         {
            {"left_hip_yaw_maximumJointTorque", "right_hip_yaw_maximumJointTorque"}, {"auto"}
         },
         {
            {"left_hip_roll_maximumJointTorque", "right_hip_roll_maximumJointTorque"}, {"auto"}
         },
         {
            {"left_hip_pitch_maximumJointTorque", "right_hip_pitch_maximumJointTorque"}, {"auto"}
         },
         {
            {"left_knee_maximumJointTorque", "right_knee_maximumJointTorque"}, {"auto"}
         },
         {
            {"left_ankle_pitch_maximumJointTorque", "right_ankle_pitch_maximumJointTorque"}, {"auto"}
         },
         {
            {"left_ankle_roll_maximumJointTorque", "right_ankle_roll_maximumJointTorque"}, {"auto"}
         },
      }, 2);


      scs.setupGraphGroup("joint power", new String[][][]
      {
         {
            {"left_state", "right_state"}, {""}
         },
         {
            {"", ""}, {"auto"}
         },
         {
            {"left_hip_yaw_jointMechPower", "right_hip_yaw_jointMechPower", "left_hip_yaw_peakMechanicalPower", "right_hip_yaw_peakMechanicalPower"}, {"auto"}
         },
         {
            {"left_hip_roll_jointMechPower", "right_hip_roll_jointMechPower", "left_hip_roll_peakMechanicalPower", "right_hip_roll_peakMechanicalPower"},
            {"auto"}
         },
         {
            {"left_hip_pitch_jointMechPower", "right_hip_pitch_jointMechPower", "left_hip_pitch_peakMechanicalPower", "right_hip_pitch_peakMechanicalPower"},
            {"auto"}
         },
         {
            {"left_knee_jointMechPower", "right_knee_jointMechPower", "left_knee_peakMechanicalPower", "right_knee_peakMechanicalPower"}, {"auto"}
         },
         {
            {"left_ankle_pitch_jointMechPower", "right_ankle_pitch_jointMechPower", "left_ankle_pitch_peakMechanicalPower",
             "right_ankle_pitch_peakMechanicalPower"}, {"auto"}
         },
         {
            {"left_ankle_roll_jointMechPower", "right_ankle_roll_jointMechPower", "left_ankle_roll_peakMechanicalPower",
             "right_ankle_roll_peakMechanicalPower"}, {"auto"}
         },
      }, 2);


      scs.setupGraphGroup("average joint power", new String[][][]
      {
         {
            {"left_state", "right_state"}, {""}
         },
         {
            {"", ""}, {"auto"}
         },
         {
            {"left_hip_yaw_averageMechanicalJointPowers", "right_hip_yaw_averageMechanicalJointPowers"}, {"auto"}
         },
         {
            {"left_hip_roll_averageMechanicalJointPowers", "right_hip_roll_averageMechanicalJointPowers"}, {"auto"}
         },
         {
            {"left_hip_pitch_averageMechanicalJointPowers", "right_hip_pitch_averageMechanicalJointPowers"}, {"auto"}
         },
         {
            {"left_knee_averageMechanicalJointPowers", "right_knee_averageMechanicalJointPowers"}, {"auto"}
         },
         {
            {"left_ankle_pitch_averageMechanicalJointPowers", "right_ankle_pitch_averageMechanicalJointPowers"}, {"auto"}
         },
         {
            {"left_ankle_roll_averageMechanicalJointPowers", "right_ankle_roll_averageMechanicalJointPowers"}, {"auto"}
         },
      }, 2);



      scs.setupConfiguration("power", "power", "power", "power");

      scs.selectConfiguration("power");
   }
   
   public String getName()
   {
      return this.name;
   }
   
   public void initialize()
   {      
   }

   public String getDescription()
   {
      return getName();
   }
}
