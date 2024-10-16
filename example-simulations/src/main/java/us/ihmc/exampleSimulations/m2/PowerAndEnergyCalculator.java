package us.ihmc.exampleSimulations.m2;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;

public class PowerAndEnergyCalculator implements RobotController
{

   private final double MOTOR_TORQUE_SENSITIVITY = 0.179;    // 0.13; //[Nm/A]
   private final double GEAR_RATIO = 50.0;    // [Nm/A]
   private final double MOTOR_RESISTANCE = 0.611;    // 100219 pdn set from RB- 02111 [ohms]

   private final YoRegistry registry = new YoRegistry("PowerCalculator");

   private final M2Robot m2Robot;

   private final YoDouble[] jointTorques;
   private final YoDouble[] jointVelocities;
   private final YoDouble[] currents;
   private final YoDouble[] gearboxEfficiencies;
   private final YoDouble[] jointPowers;
   private final YoDouble[] jointMechanicalPowers;
   private final AlphaFilteredYoVariable[] maximumJointSpeeds;
   private final AlphaFilteredYoVariable[] maximumJointTorques;


   private final AlphaFilteredYoVariable[] unsignedJointSpeeds;
   private final AlphaFilteredYoVariable[] unsignedJointTorques;


// private final YoVariable[] averageJointSpeeds;
// private final YoVariable[] averageJointTorques;
   private final YoDouble[] peakMechanicalPowers;
   private final YoDouble[] averageMechanicalJointPowers;
   private final YoDouble[] totalMechanicalJointEnergy;

   private final YoDouble totalMass;

   private final YoDouble instantaneousMechanicalPower;
   private final YoDouble averageMechanicalPower;
   private final YoDouble totalMechanicalEnergy;
   private final YoDouble mechanicalCostOfTransport;
   
   private final YoDouble totalCurrentMag;
   private final YoDouble instantaneousElectricalPower;
   private final YoDouble averageElecticalPower;
   private final YoDouble totalElectricalEnergy;
   private final YoDouble electricalCostOfTransport;
   
   private final YoDouble previousTime;
   private final YoDouble runningTime;
   private final YoDouble totalDistance;
   private final YoDouble averageWalkSpeed;
   private final YoDouble xPosition;
   private final YoDouble yPosition;
   private final YoDouble xPositionPrevious;
   private final YoDouble yPositionPrevious;
   private boolean initializedPosition = false;



   private double startTime;
   private boolean setStartTime = false;

   private final M2HarmonicDriveEfficiencyCalculator harmonicDriveEfficiencyCalculator;

   private String name;

   public PowerAndEnergyCalculator(M2Robot m2Robot, String name)
   {
      this.name = name;
      this.m2Robot = m2Robot;


      xPosition = (YoDouble)m2Robot.findVariable("q_x");
      yPosition = (YoDouble)m2Robot.findVariable("q_y");

      harmonicDriveEfficiencyCalculator = new M2HarmonicDriveEfficiencyCalculator();

      String[] prefixes = new String[] {"left_", "right_"};
      String[] listOfJointNames = new String[]
      {
         "hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_roll", "ankle_pitch"
      };

      int numberOfJoints = listOfJointNames.length * prefixes.length;
      jointTorques = new YoDouble[numberOfJoints];
      jointVelocities = new YoDouble[numberOfJoints];
      currents = new YoDouble[numberOfJoints];
      gearboxEfficiencies = new YoDouble[numberOfJoints];
      jointPowers = new YoDouble[numberOfJoints];
      jointMechanicalPowers = new YoDouble[numberOfJoints];
      maximumJointSpeeds = new AlphaFilteredYoVariable[numberOfJoints];
      maximumJointTorques = new AlphaFilteredYoVariable[numberOfJoints];


      unsignedJointSpeeds = new AlphaFilteredYoVariable[numberOfJoints];
      unsignedJointTorques = new AlphaFilteredYoVariable[numberOfJoints];


//    averageJointSpeeds = new YoVariable[numberOfJoints];
//    averageJointTorques = new YoVariable[numberOfJoints];
      peakMechanicalPowers = new YoDouble[numberOfJoints];
      averageMechanicalJointPowers = new YoDouble[numberOfJoints];
      totalMechanicalJointEnergy = new YoDouble[numberOfJoints];

      totalMass = new YoDouble("totalMass", "Total mass of the robot [kg]", registry);
 
      instantaneousMechanicalPower = new YoDouble("instantaneousMechanicalPower", "Instaneous power [W]", registry);
      averageMechanicalPower = new YoDouble("averageMechanicalPower", "Time average power [W]", registry);
      totalMechanicalEnergy = new YoDouble("totalMechanicalEnergy", "Total energy used [J]", registry);
      mechanicalCostOfTransport = new YoDouble("mechanicalCostOfTransport", "Dimensionless cost of transport not considering electrical losses", registry);
      
      totalCurrentMag = new YoDouble("totalCurrentMag", "Magnitude of total instantaneous current to all motors [A]", registry);
      instantaneousElectricalPower = new YoDouble("instantaneousElectricalPower", "Instaneous power [W]", registry);
      averageElecticalPower = new YoDouble("averageElecticalPower", "Time average power [W]", registry);
      totalElectricalEnergy = new YoDouble("totalElectricalEnergy", "Total energy used [J]", registry);
      electricalCostOfTransport = new YoDouble("electricalCostOfTransport", "Dimensionless cost of transport including electrical losses", registry);
         
      previousTime = new YoDouble("previousTime", registry);
      runningTime = new YoDouble("runningTime", registry);

      totalDistance = new YoDouble("totalDistance", registry);
      averageWalkSpeed = new YoDouble("averageWalkSpeed", registry);

      xPositionPrevious = new YoDouble("xPositionPrevious", registry);
      yPositionPrevious = new YoDouble("yPositionPrevious", registry);


      previousTime.set(m2Robot.getTime());


      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(20.0, M2Simulation.DT);

      int counter = 0;
      for (int i = 0; i < prefixes.length; i++)
      {
         for (int j = 0; j < listOfJointNames.length; j++)
         {
            String prefixedJointName = prefixes[i] + listOfJointNames[j];
            jointTorques[counter] = (YoDouble)m2Robot.findVariable("tau_" + prefixedJointName);
            jointVelocities[counter] = (YoDouble)m2Robot.findVariable("qd_" + prefixedJointName);
            currents[counter] = new YoDouble("i_" + prefixedJointName, "Instantaneous current motors [A]", registry);
            gearboxEfficiencies[counter] = new YoDouble(prefixedJointName + "_gear_eff", "Gearbox efficiency", registry);
            jointPowers[counter] = new YoDouble(prefixedJointName + "_power", "Joint power [W]", registry);

            jointMechanicalPowers[counter] = new YoDouble(prefixedJointName + "_jointMechPower", "Instantaneous joint mechanical power", registry);
            peakMechanicalPowers[counter] = new YoDouble(prefixedJointName + "_peakMechanicalPower", registry);
            averageMechanicalJointPowers[counter] = new YoDouble(prefixedJointName + "_averageMechanicalJointPowers", "", registry);
            totalMechanicalJointEnergy[counter] = new YoDouble(prefixedJointName + "_totalMechanicalJointEnergy", "", registry);

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
      Point3D comPoint = new Point3D();
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



   public YoRegistry getYoRegistry()
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
