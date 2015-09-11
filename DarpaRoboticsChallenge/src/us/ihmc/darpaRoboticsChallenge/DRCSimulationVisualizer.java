package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

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
   
   public DRCSimulationVisualizer(Robot robot, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robot = robot;
      
      YoGraphicsList yoGraphicsList = new YoGraphicsList("Simulation Viz");

      ArrayList<GroundContactPoint> groundContactPoints = robot.getAllGroundContactPoints();
      AppearanceDefinition appearance = YoAppearance.Red(); // BlackMetalMaterial();

      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         double scaleFactor = 0.0015;
         YoGraphicVector dynamicGraphicVector = new YoGraphicVector(groundContactPoint.getName(), groundContactPoint.getYoPosition(), groundContactPoint.getYoForce(), scaleFactor, appearance);
         yoGraphicsList.add(dynamicGraphicVector);
      }
      
      if (yoGraphicsListRegistry != null)
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      
      
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
