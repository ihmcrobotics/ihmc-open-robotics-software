package us.ihmc.gdx.vr;

public interface GDXVRDeviceListener
{
   /**
    * A new {@link GDXVRDevice} has connected
    **/
   void connected(GDXVRDevice device);

   /**
    * A {@link GDXVRDevice} has disconnected
    **/
   void disconnected(GDXVRDevice device);

   /**
    * A button from {@link GDXVRControllerButtons} was pressed on the {@link GDXVRDevice}
    **/
   void buttonPressed(GDXVRDevice device, int button);

   /**
    * A button from {@link GDXVRControllerButtons} was released on the {@link GDXVRDevice}
    **/
   void buttonReleased(GDXVRDevice device, int button);
}
