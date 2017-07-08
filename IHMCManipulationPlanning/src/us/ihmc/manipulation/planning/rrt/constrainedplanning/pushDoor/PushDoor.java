package us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PushDoor
{
   /*
    * doorAxisFramePose is placed on ground.
    * The Yaxis of the doorAxisFramePose is matched with door plane.
    *  
    */
   static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private FramePose doorAxisFramePose;
   private double rotationRadius;
   private double knobHeight;
   
   public PushDoor(FramePose doorAxisFramePose, double rotationRadius, double knobHeight)
   {
      this.doorAxisFramePose = doorAxisFramePose;
      this.rotationRadius = rotationRadius;
      this.knobHeight = knobHeight;
   }
   
   public PushDoor(FramePose doorAxisFramePose, Vector2D handleOffset)
   {
      this.doorAxisFramePose = doorAxisFramePose;
      this.rotationRadius = handleOffset.getX();
      this.knobHeight = handleOffset.getY();
   }
   
   public FramePose getDoorAxis()
   {
      return doorAxisFramePose;
   }
   
   public double getRadius()
   {
      return rotationRadius;
   }
   
   public double getKnobHeight()
   {
      return knobHeight;
   }
   
   public ArrayList<Graphics3DObject> getGraphics()
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();
      
      Graphics3DObject axisGraphics = new Graphics3DObject();      
      Graphics3DObject planeGraphics = new Graphics3DObject();
      Graphics3DObject knobGraphics = new Graphics3DObject();      
      
      FramePose dummyFramePose = new FramePose(doorAxisFramePose);      
      dummyFramePose.changeFrame(worldFrame);
      
      Point3D axisTranslation = new Point3D(dummyFramePose.getPosition());
      RotationMatrix axisOrientation = new RotationMatrix(dummyFramePose.getOrientation());
            
      axisGraphics.translate(axisTranslation);
      axisGraphics.rotate(axisOrientation);
      axisGraphics.addCylinder(2.0, 0.05, YoAppearance.Grey());

      Point3D planeTranslation = new Point3D(dummyFramePose.getPosition());
      RotationMatrix planeOrientation = new RotationMatrix(dummyFramePose.getOrientation());
      
      planeGraphics.translate(planeTranslation);      
      planeGraphics.rotate(planeOrientation);
      planeGraphics.translate(new Point3D(0.0, rotationRadius/2, 0.0));
      
      planeGraphics.addCube(0.05, rotationRadius, 1.5, YoAppearance.Yellow());
      
      
      Point3D knobTranslation = new Point3D(dummyFramePose.getPosition());
      RotationMatrix knobOrientation = new RotationMatrix(dummyFramePose.getOrientation());
      
      knobGraphics.translate(knobTranslation);      
      knobGraphics.rotate(knobOrientation);
      knobGraphics.translate(new Point3D(0.0, rotationRadius, knobHeight));
      
      knobGraphics.addSphere(0.03, YoAppearance.Red());
      
      ret.add(axisGraphics);
      ret.add(planeGraphics);
      ret.add(knobGraphics);
      
      return ret;
   }
}
