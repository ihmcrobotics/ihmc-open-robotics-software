package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.YoVariableRegistry;

import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class DRCSimulationVisualizer implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFramePoint simCenterOfPressure = new YoFramePoint("simCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector simGroundReactionForce = new YoFrameVector("simGroundReactionForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector simCoPMoment = new YoFrameVector("simCoPMoment", ReferenceFrame.getWorldFrame(), registry);
   
   private final Robot robot;
   
   private final Point3d copPointTemp = new Point3d();
   private final Vector3d copForceTemp = new Vector3d();
   private final Vector3d copMomentTemp = new Vector3d();
   
   public DRCSimulationVisualizer(Robot robot, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.robot = robot;
      
      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("Simulation Viz");

      ArrayList<GroundContactPoint> groundContactPoints = robot.getAllGroundContactPoints();
      AppearanceDefinition appearance = YoAppearance.Red(); // BlackMetalMaterial();

      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         double scaleFactor = 0.0015;
         DynamicGraphicVector dynamicGraphicVector = new DynamicGraphicVector(groundContactPoint.getName(), groundContactPoint, scaleFactor, appearance);
         dynamicGraphicObjectsList.add(dynamicGraphicVector);
      }
      
      if (dynamicGraphicObjectsListRegistry != null)
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      
      
      robot.setController(this, 10);
   }

   public void initialize()
   {      
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      robot.computeCenterOfPressure(copPointTemp, copForceTemp, copMomentTemp);
      
      simCenterOfPressure.set(copPointTemp);
      simCoPMoment.set(copMomentTemp);
      simGroundReactionForce.set(copForceTemp);
   }
}
