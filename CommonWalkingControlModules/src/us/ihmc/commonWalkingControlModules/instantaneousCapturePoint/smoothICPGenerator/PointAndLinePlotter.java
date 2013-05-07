package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;




import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.plotting.Artifact;
import java.awt.Color;
import java.util.ArrayList;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicPositionArtifact;
import com.yobotics.simulationconstructionset.plotting.YoFrameLineSegment2dArtifact;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLineSegment2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;


public class PointAndLinePlotter
{

   private int numberOfPoints; 
   private int numberOfLines; 
   
   private ArrayList<DynamicGraphicPosition> dynamicGraphicPositions = new ArrayList<DynamicGraphicPosition>();
   private ArrayList<DynamicGraphicPositionArtifact> dynamicGraphicPositionsArtifactList = new ArrayList<DynamicGraphicPositionArtifact>();

   private ArrayList<Artifact> lineSegmentArtifacts = new ArrayList<Artifact>();
   
   
   
   public PointAndLinePlotter()
   {
      numberOfPoints = 0; 
      numberOfLines = 0; 
   }; 
   

   
   public void addSinglePointToDynamicGraphicsObjectsListRegistry(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, String name, YoFramePoint point, AppearanceDefinition appearance, double size)
   {

      DynamicGraphicPosition dynamicGraphicPositionTemp = new DynamicGraphicPosition(name + numberOfPoints, point, size, appearance);
      dynamicGraphicPositions.add(dynamicGraphicPositionTemp); 
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("GraphicsObjects", dynamicGraphicPositions.get(numberOfPoints));
      DynamicGraphicPositionArtifact tempArtifact = dynamicGraphicPositions.get(numberOfPoints).createArtifact();
      dynamicGraphicPositionsArtifactList.add(tempArtifact);
      dynamicGraphicObjectsListRegistry.registerArtifact(name + numberOfPoints, dynamicGraphicPositionsArtifactList.get(numberOfPoints));

      numberOfPoints += 1; 
   }
   
   public void addPointListToDynamicGraphicsObjectsListRegistry(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, String name, ArrayList<YoFramePoint> pointList, AppearanceDefinition appearance, double size)
   {
      for(int i = numberOfPoints; i < numberOfPoints + pointList.size(); i++)
      {
      DynamicGraphicPosition dynamicGraphicPositionTemp = new DynamicGraphicPosition(name + (i - numberOfPoints), pointList.get(i - numberOfPoints), size, appearance);
      dynamicGraphicPositions.add(dynamicGraphicPositionTemp); 
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("GraphicsObjects", dynamicGraphicPositions.get(i));
      DynamicGraphicPositionArtifact tempArtifact = dynamicGraphicPositions.get(i).createArtifact();
      dynamicGraphicPositionsArtifactList.add(tempArtifact);
      dynamicGraphicObjectsListRegistry.registerArtifact(name + i, dynamicGraphicPositionsArtifactList.get(i));
      }
      
      numberOfPoints += pointList.size(); 
   }
   
   

   
   public void addSingleLineToDynamicGraphicsObjectsListRegistry(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, String name, YoFrameLineSegment2d lLineSegment, Color color)
   {
      Artifact lineSegmentArtifactTemp = new YoFrameLineSegment2dArtifact("line" + numberOfLines, lLineSegment, color);
      dynamicGraphicObjectsListRegistry.registerArtifact("line" + numberOfLines, lineSegmentArtifactTemp);

      numberOfLines += 1; 
   }  
   
   
   public void addLineListToDynamicGraphicsObjectsListRegistry(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, String name, ArrayList<YoFrameLineSegment2d> listOfLineSegments, Color color)
   {
      for(int i = numberOfLines; i < numberOfLines + listOfLineSegments.size(); i++)
      {
         Artifact lineSegmentArtifactTemp = new YoFrameLineSegment2dArtifact("line" + i, listOfLineSegments.get(i), color);
         lineSegmentArtifacts.add(lineSegmentArtifactTemp);
         dynamicGraphicObjectsListRegistry.registerArtifact("guideLine", lineSegmentArtifacts.get(i));
      }

      numberOfLines += listOfLineSegments.size(); 
   }
   

   
   
   

}

