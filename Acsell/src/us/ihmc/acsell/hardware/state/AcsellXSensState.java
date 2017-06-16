package us.ihmc.acsell.hardware.state;

import java.nio.ByteBuffer;

import us.ihmc.acsell.hardware.configuration.AcsellRobot;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class AcsellXSensState
{
   private final YoVariableRegistry registry;
   private final YoDouble accelX, accelY, accelZ;
   private final YoDouble gyroX, gyroY, gyroZ;
   private final YoDouble magX, magY, magZ;
   
   private final YoDouble qs, qx, qy, qz;
   
   private final YoInteger sample;
   
   private final Quaternion Qsi = new Quaternion();
   private final Quaternion Qip = new Quaternion();
   private final Quaternion Qip2 = new Quaternion();
   private final AcsellRobot robot;   
   
   
   public AcsellXSensState(String name, AcsellRobot robot, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      this.robot = robot;
      this.accelX = new YoDouble("accelX", registry);
      this.accelY = new YoDouble("accelY", registry);
      this.accelZ = new YoDouble("accelZ", registry);

      this.gyroX = new YoDouble("gyroX", registry);
      this.gyroY = new YoDouble("gyroY", registry);
      this.gyroZ = new YoDouble("gyroZ", registry);

      this.magX = new YoDouble("magX", registry);
      this.magY = new YoDouble("magY", registry);
      this.magZ = new YoDouble("magZ", registry);

      this.qs = new YoDouble("qs", registry);
      this.qx = new YoDouble("qx", registry);
      this.qy = new YoDouble("qy", registry);
      this.qz = new YoDouble("qz", registry);
      
      this.sample = new YoInteger("sample", registry);
      
      parentRegistry.addChild(registry);
   }
   
   public void getAccel(Vector3D accelToPack)
   {
      accelToPack.setX(accelX.getDoubleValue());
      accelToPack.setY(accelY.getDoubleValue());
      accelToPack.setZ(accelZ.getDoubleValue());
   }
   
   public void getGyro(Vector3D gyroToPack)
   {
      gyroToPack.setX(gyroX.getDoubleValue());
      gyroToPack.setY(gyroY.getDoubleValue());
      gyroToPack.setZ(gyroZ.getDoubleValue());
   }
   
   public void getMagnetometer(Vector3D magToPack)
   {
      magToPack.setX(magX.getDoubleValue());
      magToPack.setY(magY.getDoubleValue());
      magToPack.setZ(magZ.getDoubleValue());
   }
   
   public void getQuaternion(Quaternion quatToPack)
   {
      quatToPack.set(qx.getDoubleValue(), qy.getDoubleValue(), qz.getDoubleValue(), qs.getDoubleValue());
   }
   
   public int getSample()
   {
      return sample.getIntegerValue();
   }
   
   public void update(ByteBuffer buffer)
   {
      if(robot==AcsellRobot.STEPPR) updateSteppr(buffer);
      else updateWanderer(buffer);
   }
   
   private void updateSteppr(ByteBuffer buffer)
   {   
    //simple re-mappping of IMU Coordinates to Pelvis Coordinates  (Steve Spencer)
    //Steppr Z is IMU -X
    //Steppr Y is IMU Y
    //Steppr X is IMU Z
    accelZ.set(-buffer.getFloat());
    accelY.set(buffer.getFloat());
    accelX.set(buffer.getFloat());

    gyroZ.set(-buffer.getFloat());
    gyroY.set(buffer.getFloat());
    gyroX.set(buffer.getFloat());
    
    magZ.set(-buffer.getFloat());
    magY.set(buffer.getFloat());
    magX.set(buffer.getFloat());

    //Quaternions aren't as simple to re-map   (Steve Spencer)
    //IMU output maps IMU body coords to Spatial coords (Qsi)
    //We want to map Pelvis coords to Spatial coords (Qsp)
    //This can be done using Qsp = Qsi*Qip where Qip is a -90deg rotation about Y
    double qstemp = buffer.getFloat();
    double qxtemp = buffer.getFloat();
    double qytemp = buffer.getFloat();
    double qztemp = buffer.getFloat();
    
    Qsi.set(qxtemp,qytemp,qztemp,qstemp);
    Qip.set(0,.707106781186548,0,-.707106781186548);
    Qsi.multiply(Qip);
    qs.set(Qsi.getS());
    qx.set(Qsi.getX());
    qy.set(Qsi.getY());
    qz.set(Qsi.getZ());
    
    sample.set(buffer.getShort() & 0xFFFF);
    
    @SuppressWarnings("unused")
    int checksum = buffer.getShort() & 0xFFFF;
   }
   
   private void updateWanderer(ByteBuffer buffer)
   {
    //simple re-mappping of IMU Coordinates to Pelvis Coordinates  (Steve Spencer)
    //Wanderer Z is IMU -X
    //Wanderer Y is IMU -Y
    //Wanderer X is IMU -Z
    accelZ.set(-buffer.getFloat());
    accelY.set(-buffer.getFloat());
    accelX.set(-buffer.getFloat());

    gyroZ.set(-buffer.getFloat());
    gyroY.set(-buffer.getFloat());
    gyroX.set(-buffer.getFloat());
    
    magZ.set(-buffer.getFloat());
    magY.set(-buffer.getFloat());
    magX.set(-buffer.getFloat());

    //Quaternions aren't as simple to re-map   (Steve Spencer)
    //IMU output maps IMU body coords to Spatial coords (Qsi)
    //We want to map Pelvis coords to Spatial coords (Qsp)
    //This can be done using Qsp = Qsi*Qip where Qip is a -90deg rotation about Y
    double qstemp = buffer.getFloat();
    double qxtemp = buffer.getFloat();
    double qytemp = buffer.getFloat();
    double qztemp = buffer.getFloat();
    
    Qsi.set(qxtemp,qytemp,qztemp,qstemp);
    Qip.set(0,.707106781186548,0,-.707106781186548);
    Qip2.set(0,0,1,0);
    Qip.multiply(Qip2);
    Qsi.multiply(Qip);
    qs.set(Qsi.getS());
    qx.set(Qsi.getX());
    qy.set(Qsi.getY());
    qz.set(Qsi.getZ());
    
    sample.set(buffer.getShort() & 0xFFFF);
    
    @SuppressWarnings("unused")
    int checksum = buffer.getShort() & 0xFFFF;
   }
}
