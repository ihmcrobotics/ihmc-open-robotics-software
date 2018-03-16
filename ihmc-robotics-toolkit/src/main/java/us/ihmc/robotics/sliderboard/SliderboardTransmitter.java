package us.ihmc.robotics.sliderboard;

import javax.sound.midi.Receiver;

public class SliderboardTransmitter
{
   private final Receiver reciever;

   public SliderboardTransmitter(Receiver reciever)
   {
      this.reciever = reciever;
   }

//   public void send(double percent, int channel)
//   {
//      ShortMessage shortMesssage = new ShortMessage();
//      shortMesssage.setMessage(176, 0, channel + 80, );
//   }
}
