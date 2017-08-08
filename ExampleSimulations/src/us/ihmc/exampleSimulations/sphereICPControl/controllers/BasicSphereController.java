package us.ihmc.exampleSimulations.sphereICPControl.controllers;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BasicSphereController implements GenericSphereController
{
   private final BasicHeightController heightController;
   private final BasicPlanarController planarController;

   public BasicSphereController(SphereControlToolbox controlToolbox, YoVariableRegistry registry)
   {
      heightController = new BasicHeightController(controlToolbox, registry);
      planarController = new BasicPlanarController(controlToolbox, registry);
   }

   private final Point2D planarForces = new Point2D();
   private final Vector3D forces = new Vector3D();
   public void doControl()
   {
      heightController.doControl();
      planarController.doControl();
      planarController.getPlanarForces(planarForces);

      forces.set(planarForces.getX(), planarForces.getY(), heightController.getVerticalForce());
   }

   public Vector3D getForces()
   {
      return forces;
   }
}
