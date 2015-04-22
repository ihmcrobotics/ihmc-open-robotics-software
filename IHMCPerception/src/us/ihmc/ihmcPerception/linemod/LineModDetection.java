package us.ihmc.ihmcPerception.linemod;

public class LineModDetection
{
     /** \brief x-position of the detection. */
     int x;
     /** \brief y-position of the detection. */
     int y;
     /** \brief ID of the detected template. */
     int template_id;
     /** \brief score of the detection. */
     float score;
     /** \brief scale at which the template was detected. */
     float scale;
     
     public String toString()
     {
        return "x " + x + " y " + y + " template id " + template_id + " score " + " scale " + scale;
     }
}
