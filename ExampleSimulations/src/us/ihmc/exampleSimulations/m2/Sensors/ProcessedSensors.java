package us.ihmc.exampleSimulations.m2.Sensors;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.exampleSimulations.m2.ContactPointName;
import us.ihmc.exampleSimulations.m2.JointName;
import us.ihmc.exampleSimulations.m2.RobotAxis;
import us.ihmc.exampleSimulations.m2.RobotOrientation;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * <p>
 * ProcessedSensors
 * </p>
 *
 * <p>
 * Class holding onto the sensor information as it is processed. Various
 * processing classes will take this and do extra processing to it.
 * </p>
 *
 * @author IHMC ProjectM Team
 */
public class ProcessedSensors
{
   private final YoVariableRegistry registry = new YoVariableRegistry("ProcessedSensors");

   public final DoubleYoVariable time = new DoubleYoVariable("time", registry);

   private final DoubleYoVariable p_x_world = new DoubleYoVariable("p_x_world", registry);
   private final DoubleYoVariable p_y_world = new DoubleYoVariable("p_y_world", registry);
   private final DoubleYoVariable p_z_world = new DoubleYoVariable("p_z_world", registry);

   private final DoubleYoVariable[] robotBodyPositionInWorldcoordinates = new DoubleYoVariable[] {p_x_world, p_y_world, p_z_world};

   private final DoubleYoVariable pd_x_world = new DoubleYoVariable("pd_x_world", registry);
   private final DoubleYoVariable pd_y_world = new DoubleYoVariable("pd_y_world", registry);
   private final DoubleYoVariable pd_z_world = new DoubleYoVariable("pd_z_world", registry);

   private final DoubleYoVariable[] robotBodyVelocityInWorldcoordinates = new DoubleYoVariable[] {pd_x_world, pd_y_world, pd_z_world};

   private final DoubleYoVariable pdd_x_world = new DoubleYoVariable("pdd_x_world", registry);
   private final DoubleYoVariable pdd_y_world = new DoubleYoVariable("pdd_y_world", registry);
   private final DoubleYoVariable pdd_z_world = new DoubleYoVariable("pdd_z_world", registry);

   private final DoubleYoVariable[] robotBodyAccelerationInWorldcoordinates = new DoubleYoVariable[] {pdd_x_world, pdd_y_world, pdd_z_world};

   private final DoubleYoVariable p_yaw = new DoubleYoVariable("p_yaw", registry);
   private final DoubleYoVariable p_roll = new DoubleYoVariable("p_roll", registry);
   private final DoubleYoVariable p_pitch = new DoubleYoVariable("p_pitch", registry);

   private final DoubleYoVariable pd_yaw = new DoubleYoVariable("pd_yaw", registry);
   private final DoubleYoVariable pd_roll = new DoubleYoVariable("pd_roll", registry);
   private final DoubleYoVariable pd_pitch = new DoubleYoVariable("pd_pitch", registry);

   private final DoubleYoVariable pdd_yaw = new DoubleYoVariable("pdd_yaw", registry);
   private final DoubleYoVariable pdd_roll = new DoubleYoVariable("pdd_roll", registry);
   private final DoubleYoVariable pdd_pitch = new DoubleYoVariable("pdd_pitch", registry);

   // p Joint positions in Units of Radians:
   private final DoubleYoVariable p_left_hip_yaw = new DoubleYoVariable("p_left_hip_yaw", registry);
   private final DoubleYoVariable p_left_hip_roll = new DoubleYoVariable("p_left_hip_roll", registry);
   private final DoubleYoVariable p_left_hip_pitch = new DoubleYoVariable("p_left_hip_pitch", registry);
   private final DoubleYoVariable p_left_knee = new DoubleYoVariable("p_left_knee", registry);
   private final DoubleYoVariable p_left_ankle_pitch = new DoubleYoVariable("p_left_ankle_pitch", registry);
   private final DoubleYoVariable p_left_ankle_roll = new DoubleYoVariable("p_left_ankle_roll", registry);

   private final DoubleYoVariable p_right_hip_yaw = new DoubleYoVariable("p_right_hip_yaw", registry);
   private final DoubleYoVariable p_right_hip_roll = new DoubleYoVariable("p_right_hip_roll", registry);
   private final DoubleYoVariable p_right_hip_pitch = new DoubleYoVariable("p_right_hip_pitch", registry);
   private final DoubleYoVariable p_right_knee = new DoubleYoVariable("p_right_knee", registry);
   private final DoubleYoVariable p_right_ankle_pitch = new DoubleYoVariable("p_right_ankle_pitch", registry);
   private final DoubleYoVariable p_right_ankle_roll = new DoubleYoVariable("p_right_ankle_roll", registry);

   // **pd

   private final DoubleYoVariable pd_left_hip_yaw = new DoubleYoVariable("pd_left_hip_yaw", registry);
   private final DoubleYoVariable pd_left_hip_roll = new DoubleYoVariable("pd_left_hip_roll", registry);
   private final DoubleYoVariable pd_left_hip_pitch = new DoubleYoVariable("pd_left_hip_pitch", registry);
   private final DoubleYoVariable pd_left_knee = new DoubleYoVariable("pd_left_knee", registry);
   private final DoubleYoVariable pd_left_ankle_pitch = new DoubleYoVariable("pd_left_ankle_pitch", registry);
   private final DoubleYoVariable pd_left_ankle_roll = new DoubleYoVariable("pd_left_ankle_roll", registry);

   private final DoubleYoVariable pd_right_hip_yaw = new DoubleYoVariable("pd_right_hip_yaw", registry);
   private final DoubleYoVariable pd_right_hip_roll = new DoubleYoVariable("pd_right_hip_roll", registry);
   private final DoubleYoVariable pd_right_hip_pitch = new DoubleYoVariable("pd_right_hip_pitch", registry);
   private final DoubleYoVariable pd_right_knee = new DoubleYoVariable("pd_right_knee", registry);
   private final DoubleYoVariable pd_right_ankle_pitch = new DoubleYoVariable("pd_right_ankle_pitch", registry);
   private final DoubleYoVariable pd_right_ankle_roll = new DoubleYoVariable("pd_right_ankle_roll", registry);

   // pdd

   private final DoubleYoVariable pdd_left_hip_yaw = new DoubleYoVariable("pdd_left_hip_yaw", registry);
   private final DoubleYoVariable pdd_left_hip_roll = new DoubleYoVariable("pdd_left_hip_roll", registry);
   private final DoubleYoVariable pdd_left_hip_pitch = new DoubleYoVariable("pdd_left_hip_pitch", registry);
   private final DoubleYoVariable pdd_left_knee = new DoubleYoVariable("pdd_left_knee", registry);
   private final DoubleYoVariable pdd_left_ankle_pitch = new DoubleYoVariable("pdd_left_ankle_pitch", registry);
   private final DoubleYoVariable pdd_left_ankle_roll = new DoubleYoVariable("pdd_left_ankle_roll", registry);

   private final DoubleYoVariable pdd_right_hip_yaw = new DoubleYoVariable("pdd_right_hip_yaw", registry);
   private final DoubleYoVariable pdd_right_hip_roll = new DoubleYoVariable("pdd_right_hip_roll", registry);
   private final DoubleYoVariable pdd_right_hip_pitch = new DoubleYoVariable("pdd_right_hip_pitch", registry);
   private final DoubleYoVariable pdd_right_knee = new DoubleYoVariable("pdd_right_knee", registry);
   private final DoubleYoVariable pdd_right_ankle_pitch = new DoubleYoVariable("pdd_right_ankle_pitch", registry);
   private final DoubleYoVariable pdd_right_ankle_roll = new DoubleYoVariable("pdd_right_ankle_roll", registry);

   // ground contact points
   private final DoubleYoVariable p_right_toe_in_x = new DoubleYoVariable("p_right_toe_in_x", registry);
   private final DoubleYoVariable p_right_toe_in_y = new DoubleYoVariable("p_right_toe_in_y", registry);
   private final DoubleYoVariable p_right_toe_in_z = new DoubleYoVariable("p_right_toe_in_z", registry);
   private final DoubleYoVariable p_right_toe_out_x = new DoubleYoVariable("p_right_toe_out_x", registry);
   private final DoubleYoVariable p_right_toe_out_y = new DoubleYoVariable("p_right_toe_out_y", registry);
   private final DoubleYoVariable p_right_toe_out_z = new DoubleYoVariable("p_right_toe_out_z", registry);

   private final DoubleYoVariable p_right_heel_in_x = new DoubleYoVariable("p_right_heel_in_x", registry);
   private final DoubleYoVariable p_right_heel_in_y = new DoubleYoVariable("p_right_heel_in_y", registry);
   private final DoubleYoVariable p_right_heel_in_z = new DoubleYoVariable("p_right_heel_in_z", registry);
   private final DoubleYoVariable p_right_heel_out_x = new DoubleYoVariable("p_right_heel_out_x", registry);
   private final DoubleYoVariable p_right_heel_out_y = new DoubleYoVariable("p_right_heel_out_y", registry);
   private final DoubleYoVariable p_right_heel_out_z = new DoubleYoVariable("p_right_heel_out_z", registry);

   private final DoubleYoVariable p_left_toe_in_x = new DoubleYoVariable("p_left_toe_in_x", registry);
   private final DoubleYoVariable p_left_toe_in_y = new DoubleYoVariable("p_left_toe_in_y", registry);
   private final DoubleYoVariable p_left_toe_in_z = new DoubleYoVariable("p_left_toe_in_z", registry);
   private final DoubleYoVariable p_left_toe_out_x = new DoubleYoVariable("p_left_toe_out_x", registry);
   private final DoubleYoVariable p_left_toe_out_y = new DoubleYoVariable("p_left_toe_out_y", registry);
   private final DoubleYoVariable p_left_toe_out_z = new DoubleYoVariable("p_left_toe_out_z", registry);

   private final DoubleYoVariable p_left_heel_in_x = new DoubleYoVariable("p_left_heel_in_x", registry);
   private final DoubleYoVariable p_left_heel_in_y = new DoubleYoVariable("p_left_heel_in_y", registry);
   private final DoubleYoVariable p_left_heel_in_z = new DoubleYoVariable("p_left_heel_in_z", registry);
   private final DoubleYoVariable p_left_heel_out_x = new DoubleYoVariable("p_left_heel_out_x", registry);
   private final DoubleYoVariable p_left_heel_out_y = new DoubleYoVariable("p_left_heel_out_y", registry);
   private final DoubleYoVariable p_left_heel_out_z = new DoubleYoVariable("p_left_heel_out_z", registry);

   private final DoubleYoVariable p_right_toe_in_fx = new DoubleYoVariable("p_right_toe_in_fx", registry);
   private final DoubleYoVariable p_right_toe_in_fy = new DoubleYoVariable("p_right_toe_in_fy", registry);
   private final DoubleYoVariable p_right_toe_in_fz = new DoubleYoVariable("p_right_toe_in_fz", registry);
   private final DoubleYoVariable p_right_toe_out_fx = new DoubleYoVariable("p_right_toe_out_fx", registry);
   private final DoubleYoVariable p_right_toe_out_fy = new DoubleYoVariable("p_right_toe_out_fy", registry);
   private final DoubleYoVariable p_right_toe_out_fz = new DoubleYoVariable("p_right_toe_out_fz", registry);

   private final DoubleYoVariable p_right_heel_in_fx = new DoubleYoVariable("p_right_heel_in_fx", registry);
   private final DoubleYoVariable p_right_heel_in_fy = new DoubleYoVariable("p_right_heel_in_fy", registry);
   private final DoubleYoVariable p_right_heel_in_fz = new DoubleYoVariable("p_right_heel_in_fz", registry);
   private final DoubleYoVariable p_right_heel_out_fx = new DoubleYoVariable("p_right_heel_out_fx", registry);
   private final DoubleYoVariable p_right_heel_out_fy = new DoubleYoVariable("p_right_heel_out_fy", registry);
   private final DoubleYoVariable p_right_heel_out_fz = new DoubleYoVariable("p_right_heel_out_fz", registry);

   private final DoubleYoVariable p_left_toe_in_fx = new DoubleYoVariable("p_left_toe_in_fx", registry);
   private final DoubleYoVariable p_left_toe_in_fy = new DoubleYoVariable("p_left_toe_in_fy", registry);
   private final DoubleYoVariable p_left_toe_in_fz = new DoubleYoVariable("p_left_toe_in_fz", registry);
   private final DoubleYoVariable p_left_toe_out_fx = new DoubleYoVariable("p_left_toe_out_fx", registry);
   private final DoubleYoVariable p_left_toe_out_fy = new DoubleYoVariable("p_left_toe_out_fy", registry);
   private final DoubleYoVariable p_left_toe_out_fz = new DoubleYoVariable("p_left_toe_out_fz", registry);

   private final DoubleYoVariable p_left_heel_in_fx = new DoubleYoVariable("p_left_heel_in_fx", registry);
   private final DoubleYoVariable p_left_heel_in_fy = new DoubleYoVariable("p_left_heel_in_fy", registry);
   private final DoubleYoVariable p_left_heel_in_fz = new DoubleYoVariable("p_left_heel_in_fz", registry);
   private final DoubleYoVariable p_left_heel_out_fx = new DoubleYoVariable("p_left_heel_out_fx", registry);
   private final DoubleYoVariable p_left_heel_out_fy = new DoubleYoVariable("p_left_heel_out_fy", registry);
   private final DoubleYoVariable p_left_heel_out_fz = new DoubleYoVariable("p_left_heel_out_fz", registry);

   private final DoubleYoVariable p_left_toe_in_fs = new DoubleYoVariable("p_left_toe_in_fs", registry);
   private final DoubleYoVariable p_left_toe_out_fs = new DoubleYoVariable("p_left_toe_out_fs", registry);
   private final DoubleYoVariable p_left_heel_in_fs = new DoubleYoVariable("p_left_heel_in_fs", registry);
   private final DoubleYoVariable p_left_heel_out_fs = new DoubleYoVariable("p_left_heel_out_fs", registry);

   private final DoubleYoVariable p_right_toe_in_fs = new DoubleYoVariable("p_right_toe_in_fs", registry);
   private final DoubleYoVariable p_right_toe_out_fs = new DoubleYoVariable("p_right_toe_out_fs", registry);
   private final DoubleYoVariable p_right_heel_in_fs = new DoubleYoVariable("p_right_heel_in_fs", registry);
   private final DoubleYoVariable p_right_heel_out_fs = new DoubleYoVariable("p_right_heel_out_fs", registry);

   private final DoubleYoVariable[][] groundContactPointsPositions = new DoubleYoVariable[][]
   {
      {
         p_left_toe_in_x, p_left_toe_in_y, p_left_toe_in_z, p_left_toe_out_x, p_left_toe_out_y, p_left_toe_out_z, p_left_heel_in_x, p_left_heel_in_y,
         p_left_heel_in_z, p_left_heel_out_x, p_left_heel_out_y, p_left_heel_out_z
      },
      {
         p_right_toe_in_x, p_right_toe_in_y, p_right_toe_in_z, p_right_toe_out_x, p_right_toe_out_y, p_right_toe_out_z, p_right_heel_in_x, p_right_heel_in_y,
         p_right_heel_in_z, p_right_heel_out_x, p_right_heel_out_y, p_right_heel_out_z,
      }
   };

   private final DoubleYoVariable[][] groundContactPointsForces = new DoubleYoVariable[][]
   {
      {
         p_left_toe_in_fx, p_left_toe_in_fy, p_left_toe_in_fz, p_left_toe_out_fx, p_left_toe_out_fy, p_left_toe_out_fz, p_left_heel_in_fx, p_left_heel_in_fy,
         p_left_heel_in_fz, p_left_heel_out_fx, p_left_heel_out_fy, p_left_heel_out_fz
      },
      {
         p_right_toe_in_fx, p_right_toe_in_fy, p_right_toe_in_fz, p_right_toe_out_fx, p_right_toe_out_fy, p_right_toe_out_fz, p_right_heel_in_fx,
         p_right_heel_in_fy, p_right_heel_in_fz, p_right_heel_out_fx, p_right_heel_out_fy, p_right_heel_out_fz,
      }
   };

   private final DoubleYoVariable[][] groundContactPointsFootSwitch = new DoubleYoVariable[][]
   {
      {p_left_toe_in_fs, p_left_toe_out_fs, p_left_heel_in_fs, p_left_heel_out_fs},
      {p_right_toe_in_fs, p_right_toe_out_fs, p_right_heel_in_fs, p_right_heel_out_fs}
   };

   private boolean leftFootOnGround, rightFootOnGround;

   private final DoubleYoVariable[] robotYawPitchOrRoll = new DoubleYoVariable[] {p_yaw, p_pitch, p_roll};
   private final DoubleYoVariable[] robotYawPitchOrRollVelocity = new DoubleYoVariable[] {pd_yaw, pd_pitch, pd_roll};
   private final DoubleYoVariable[] robotYawPitchOrRollAcceleration = new DoubleYoVariable[] {pdd_yaw, pdd_pitch, pdd_roll};

   private final DoubleYoVariable[][] limbJointPositions = new DoubleYoVariable[][]
   {
      {
         p_left_hip_yaw, p_left_hip_roll, p_left_hip_pitch, p_left_knee, p_left_ankle_pitch, p_left_ankle_roll
      },
      {
         p_right_hip_yaw, p_right_hip_roll, p_right_hip_pitch, p_right_knee, p_right_ankle_pitch, p_right_ankle_roll
      }
   };

   private final DoubleYoVariable[][] limbJointVelocities = new DoubleYoVariable[][]
   {
      {
         pd_left_hip_yaw, pd_left_hip_roll, pd_left_hip_pitch, pd_left_knee, pd_left_ankle_pitch, pd_left_ankle_roll
      },
      {
         pd_right_hip_yaw, pd_right_hip_roll, pd_right_hip_pitch, pd_right_knee, pd_right_ankle_pitch, pd_right_ankle_roll
      }
   };

   private final DoubleYoVariable[][] limbJointAccelerations = new DoubleYoVariable[][]
   {
      {
         pdd_left_hip_yaw, pdd_left_hip_roll, pdd_left_hip_pitch, pdd_left_knee, pdd_left_ankle_pitch, pdd_left_ankle_roll
      },
      {
         pdd_right_hip_yaw, pdd_right_hip_roll, pdd_right_hip_pitch, pdd_right_knee, pdd_right_ankle_pitch, pdd_right_ankle_roll
      }
   };

   private final DoubleYoVariable[] allVariables;

   public void setGroundContactPointsPositions(Point3D[][] groundContactFramePointsPositions)
   {
      for (int i = 0; i < RobotSide.values().length; i++)
      {
         for (int j = 0; j < ContactPointName.values().length; j++)
         {
            this.groundContactPointsPositions[i][j * 3 + 0].set(groundContactFramePointsPositions[i][j].getX());
            this.groundContactPointsPositions[i][j * 3 + 1].set(groundContactFramePointsPositions[i][j].getY());
            this.groundContactPointsPositions[i][j * 3 + 2].set(groundContactFramePointsPositions[i][j].getZ());
         }
      }
   }

   public void setGroundContactPointsForces(Point3D[][] groundContactFramePointsForces)
   {
      for (int i = 0; i < RobotSide.values().length; i++)
      {
         for (int j = 0; j < ContactPointName.values().length; j++)
         {
            this.groundContactPointsForces[i][j * 3 + 0].set(groundContactFramePointsForces[i][j].getX());
            this.groundContactPointsForces[i][j * 3 + 1].set(groundContactFramePointsForces[i][j].getY());
            this.groundContactPointsForces[i][j * 3 + 2].set(groundContactFramePointsForces[i][j].getZ());
         }
      }
   }

   public void setTime(double time)
   {
      this.time.set(time);
   }

   public void setGroundContactPointFootSwitch(RobotSide robotSide, double[] footSwitches)
   {
      copyValuesIntoYoVariableArray(groundContactPointsFootSwitch[robotSide.ordinal()], footSwitches);
   }

   public void setRobotBodyPositionInWorldcoordinates(double[] bodyPositionInWorldcoordinatesXYZ)
   {
      copyValuesIntoYoVariableArray(robotBodyPositionInWorldcoordinates, bodyPositionInWorldcoordinatesXYZ);
   }

   public void setRobotBodyVelocityInWorldCoordiantes(double[] bodyVelocityInWorldCoodinatesXYZ)
   {
      copyValuesIntoYoVariableArray(robotBodyVelocityInWorldcoordinates, bodyVelocityInWorldCoodinatesXYZ);
   }

   public void setRobotBodyAccelerationInWorldcoordinates(double[] bodyAccelerationInWorldcoordinatesXYZ)
   {
      copyValuesIntoYoVariableArray(robotBodyAccelerationInWorldcoordinates, bodyAccelerationInWorldcoordinatesXYZ);
   }

   public void setRobotYawPitchAndRoll(double[] bodyAngles)
   {
      copyValuesIntoYoVariableArray(robotYawPitchOrRoll, bodyAngles);

   }

   public void setRobotYawPitchAndRollVelocity(double[] bodyVelocity)
   {
      copyValuesIntoYoVariableArray(robotYawPitchOrRollVelocity, bodyVelocity);

   }

   public void setRobotYawPitchAndRollAcceleration(double[] bodyAcceleration)
   {
      copyValuesIntoYoVariableArray(robotYawPitchOrRollAcceleration, bodyAcceleration);

   }

   public void setLimbJointAngles(RobotSide robotSide, double[] jointAngles)
   {
      DoubleYoVariable[] jointAngleYoVariables = limbJointPositions[robotSide.ordinal()];

      copyValuesIntoYoVariableArray(jointAngleYoVariables, jointAngles);

   }

   public void setLimbJointVelocities(RobotSide robotSide, double[] jointVelocities)
   {
      DoubleYoVariable[] jointVelocityYoVariables = limbJointVelocities[robotSide.ordinal()];

      copyValuesIntoYoVariableArray(jointVelocityYoVariables, jointVelocities);

   }

   public void setLimbJointAccelerations(RobotSide robotSide, double[] jointAcceleration)
   {
      DoubleYoVariable[] jointVelocityYoVariables = limbJointAccelerations[robotSide.ordinal()];


      copyValuesIntoYoVariableArray(jointVelocityYoVariables, jointAcceleration);


   }

   public void setJointAngles(double[][] jointAngles)
   {
      for (int i = 0; i < limbJointPositions.length; i++)
      {
         for (int j = 0; j < limbJointPositions[i].length; j++)
         {
            limbJointPositions[i][j].set(jointAngles[i][j]);
         }
      }
   }


   public void setLeftFootOnGround(boolean onGround)
   {
      leftFootOnGround = onGround;
   }

   public void setRightFootOnGround(boolean onGround)
   {
      rightFootOnGround = onGround;
   }

   public boolean isLeftFootOnGround()
   {
      return leftFootOnGround;
   }

   public boolean isRightFootOnGround()
   {
      return rightFootOnGround;
   }


   public double getTime()
   {
      return time.getDoubleValue();
   }

   public DoubleYoVariable getTimeYoVariable()
   {
      return time;
   }

   public Point3D getGroundContactPointsPositions(RobotSide robotSide, ContactPointName contactPointName)
   {
      return new Point3D(this.groundContactPointsPositions[robotSide.ordinal()][contactPointName.ordinal() * 3 + 0].getDoubleValue(),
                         this.groundContactPointsPositions[robotSide.ordinal()][contactPointName.ordinal() * 3 + 1].getDoubleValue(),
                         this.groundContactPointsPositions[robotSide.ordinal()][contactPointName.ordinal() * 3 + 2].getDoubleValue());
   }

   // +++ 100121 pdn: This should be cleaned up.
   public Point3D getGroundContactPointForces(RobotSide robotSide, ContactPointName contactPointName)
   {
      return new Point3D(this.groundContactPointsForces[robotSide.ordinal()][contactPointName.ordinal() * 3 + 0].getDoubleValue(),
                         this.groundContactPointsForces[robotSide.ordinal()][contactPointName.ordinal() * 3 + 1].getDoubleValue(),
                         this.groundContactPointsForces[robotSide.ordinal()][contactPointName.ordinal() * 3 + 2].getDoubleValue());
   }

   public double getTotalFootForce(RobotSide robotSide)
   {
      double ret = 0.0;
      for (ContactPointName pointName : ContactPointName.values())
      {
         ret += getGroundContactPointForces(robotSide, pointName).getZ();
      }

      return ret;
   }

   public double getGroundContactPointFootSwitch(RobotSide robotSide, ContactPointName contactPointName)
   {
      return groundContactPointsFootSwitch[robotSide.ordinal()][contactPointName.ordinal()].getDoubleValue();
   }

   public DoubleYoVariable getLimbJointPositionYoVariable(RobotSide robotSide, JointName jointName)
   {
      return limbJointPositions[robotSide.ordinal()][jointName.ordinal()];
   }

   public DoubleYoVariable getLimbJointVelocityYoVariable(RobotSide robotSide, JointName jointName)
   {
      return limbJointVelocities[robotSide.ordinal()][jointName.ordinal()];
   }

   public DoubleYoVariable getLimbJointAccelerationYoVariable(RobotSide robotSide, JointName jointName)
   {
      return limbJointAccelerations[robotSide.ordinal()][jointName.ordinal()];
   }

   public double getRobotYawPitchOrRoll(RobotOrientation orientation)
   {
      return robotYawPitchOrRoll[orientation.ordinal()].getDoubleValue();
   }

   public double getRobotYawPitchOrRollVelocity(RobotOrientation orientation)
   {
      return robotYawPitchOrRollVelocity[orientation.ordinal()].getDoubleValue();
   }

   public double getRobotYawPitchOrRollAcceleration(RobotOrientation orientation)
   {
      return robotYawPitchOrRollAcceleration[orientation.ordinal()].getDoubleValue();
   }

   public double getRobotBodyPositionComponentInWorldCoordinates(RobotAxis axis)
   {
      return robotBodyPositionInWorldcoordinates[axis.ordinal()].getDoubleValue();
   }

   public double getRobotBodyVelocityComponentInWorldCoordinates(RobotAxis axis)
   {
      return robotBodyVelocityInWorldcoordinates[axis.ordinal()].getDoubleValue();
   }

   public double getRobotBodyAccelerationComponentInWorldCoordinates(RobotAxis axis)
   {
      return robotBodyAccelerationInWorldcoordinates[axis.ordinal()].getDoubleValue();
   }

   public double getJointPosition(RobotSide robotSide, JointName jointName)
   {
      return limbJointPositions[robotSide.ordinal()][jointName.ordinal()].getDoubleValue();
   }

   public double getJointVelocity(RobotSide robotSide, JointName jointName)
   {
      return limbJointVelocities[robotSide.ordinal()][jointName.ordinal()].getDoubleValue();
   }

   public double getJointAcceleration(RobotSide robotSide, JointName jointName)
   {
      return limbJointAccelerations[robotSide.ordinal()][jointName.ordinal()].getDoubleValue();
   }

   public void snapshot(ProcessedSensors processedSensors)
   {
      for (int i = 0; i < allVariables.length; i++)
      {
         allVariables[i].set(processedSensors.allVariables[i].getDoubleValue());
      }
   }

   public String toString()
   {
      String ret = "";

      for (RobotSide robotSide : RobotSide.values())
      {
         for (JointName jointName : JointName.values())
         {
            DoubleYoVariable jointPosition = limbJointPositions[robotSide.ordinal()][jointName.ordinal()];

            ret = ret + jointPosition.toString() + " = " + jointPosition.getDoubleValue() + "\n";
         }
      }

      return ret;
   }

   private void copyValuesIntoYoVariableArray(DoubleYoVariable[] yoVariablesTarget, double[] valuesSource)
   {
      if (yoVariablesTarget.length != valuesSource.length)
      {
         throw new RuntimeException("Arrays must be the same length yoVariablesTarget length: " + yoVariablesTarget.length + " valuesSource length: "
                                    + valuesSource.length);
      }

      for (int i = 0; i < valuesSource.length; i++)
      {
         yoVariablesTarget[i].set(valuesSource[i]);
      }
   }

   public ProcessedSensors()
   {
      ArrayList<YoVariable<?>> variables = registry.getAllVariablesIncludingDescendants();
      allVariables = new DoubleYoVariable[variables.size()];
      variables.toArray(allVariables);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public static void setupGUI(SimulationConstructionSet scs)
   {
      scs.setupGraphGroup("Processed IMU", new String[][]
      {
         {"p_yaw_world", "p_pitch_world", "p_roll_world"}, {"pd_wx_world", "pd_wy_world", "pd_wz_world"},
         {"p_qs_world", "p_qx_world", "p_qy_world", "p_qz_world"}, {"p_compass_x", "p_compass_y", "p_compass_z"}, {"p_x_world", "p_y_world", "p_z_world"}
      }, 1);

      scs.setupGraphGroup("Processed Robot Amperages", new String[][]
      {
         {"p_lh_yaw_amp", "p_rh_yaw_amp"}, {"p_lh_roll_amp", "p_rh_roll_amp"}, {"p_lh_pitch_amp", "p_rh_pitch_amp"}, {"p_lk_amp", "p_rk_amp"},
         {"p_la_in_amp", "p_ra_in_amp"}, {"p_la_out_amp", "p_ra_out_amp"},
      }, 1);

      scs.setupGraphGroup("Processed Magnetometer Orientation", new String[][]
      {
         {"p_yaw_mag"}, {"p_roll_mag"}, {"p_pitch_mag"},
      }, 1);

      scs.setupGraphGroup("Processed Orientations", new String[][]
      {
         {"p_yaw_magnet"}, {"p_roll_magnet"}, {"p_pitch_magnet"},
      }, 1);

      scs.setupGraphGroup("Processed Orientations2", new String[][]
      {
         {"p_yaw_magnet"}, {"p_roll_magnet"}, {"p_pitch_magnet"}, {"q_pitch"}, {"q_roll"},
      }, 1);

   }

}
