package us.ihmc.avatar.roughTerrainWalking;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class Step
{
   private static final AppearanceDefinition steppingStoneColor = YoAppearance.DarkGray();
   private final double stepSizeInY;
   private final double stepSizeInZ;
   private final double stepSizeInX;

   private Point3D centerOfStep;
   AxisAngle orinetationOfStep;

   private double centreOfStepInX;
   private double centreOfStepInY;
   private double centreOfStepInZ;
   private double yawInRadians;



   public Step(double stepSizeX, double stepSizeY, double stepSizeZ, double centerOfStepInXAxis, double centerOfStepInYAxis, double centerOfStepInZAxis, double yawInRadians)
   {
      // TODO Auto-generated constructor stub
      this.centreOfStepInX = centerOfStepInXAxis;
      this.centreOfStepInY = centerOfStepInYAxis;
      this.centreOfStepInZ = centerOfStepInZAxis;
      this.yawInRadians =yawInRadians;
      this.stepSizeInX=stepSizeX;
      this.stepSizeInY=stepSizeY;
      this.stepSizeInZ=stepSizeZ;
   }

   private Box3D stepAsBox3DObject()
   {
      orinetationOfStep=new AxisAngle(yawInRadians,0,0);
      centerOfStep = new Point3D(centreOfStepInX, centreOfStepInY, centreOfStepInZ);
      Box3D stepAsBox3DObject = new Box3D(centerOfStep, orinetationOfStep,stepSizeInX,stepSizeInY, stepSizeInZ );
      return stepAsBox3DObject;
   }

   public TerrainObject3D createStepAsTerrainObject()
   {
      Box3D step = stepAsBox3DObject();
      TerrainObject3D stepAsTerrain3DObject=new RotatableBoxTerrainObject(step, steppingStoneColor);
      return stepAsTerrain3DObject;
   }

}