package us.ihmc.sensorProcessing.pointClouds;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.thoughtworks.xstream.XStream;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;

/**
 * @author Peter Abeles
 */
public class GeometryOps {

   public static Transform convert( Se3_F64 input , Transform output ) {
      if( output == null )
         output = new Transform();

      Quaternion_F64 quatGR = new Quaternion_F64();

      RotationMatrixGenerator.matrixToQuaternion(input.getR(),quatGR);

      Quaternion quatJME = new Quaternion();
      quatJME.set((float)quatGR.x,(float)quatGR.y,(float)quatGR.z,(float)quatGR.w);

      output.setRotation(quatJME);
      output.setTranslation((float)input.T.x,(float)input.T.y,(float)input.T.z);

      return output;
   }

   public static Se3_F64 convert( Transform input , Se3_F64 output ) {

      if( output == null )
         output = new Se3_F64();

      Quaternion quatJME = input.getRotation();
      Vector3f tranJME = input.getTranslation();

      Quaternion_F64 quatGR = new Quaternion_F64();
      quatGR.x = quatJME.getX();
      quatGR.y = quatJME.getY();
      quatGR.z = quatJME.getZ();
      quatGR.w = quatJME.getW();

      RotationMatrixGenerator.quaternionToMatrix(quatGR, output.getR());
      output.getT().set(tranJME.x,tranJME.y,tranJME.z);

      try {
         new XStream().toXML(output,new FileOutputStream("testbedToWorld.xml"));
      } catch (FileNotFoundException e) {
         throw new RuntimeException(e);
      }

      return output;
   }
}
